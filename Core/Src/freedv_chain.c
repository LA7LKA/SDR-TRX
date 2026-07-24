#include "freedv_chain.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f7xx.h"      /* must precede arm_math.h: defines __FPU_PRESENT */
#include "arm_math.h"
#include "codec2_fifo.h"
#include "freedv_api.h"

/*
 * Building codec2 with __EMBEDDED__ makes it call these instead of malloc()
 * directly, so the application decides where its memory comes from. Plain
 * newlib heap is fine here; _sbrk() already bounds it against the MSP stack.
 */
void *codec2_malloc(size_t size)               { return malloc(size); }
void *codec2_calloc(size_t nmemb, size_t size) { return calloc(nmemb, size); }
void  codec2_free(void *ptr)                   { free(ptr); }

/* 48 kHz -> 8 kHz */
#define DECIM_M      6
#define DECIM_TAPS   97

/*
 * Decimate in fixed chunks so arm_fir_decimate_f32 always gets a block length
 * that is a multiple of M. The 48 kHz side pushes 1024 samples per DMA half,
 * which is not a multiple of 6, so whatever does not fill a chunk is carried
 * over to the next call.
 */
#define DECIM_IN_CHUNK   960                        /* 20 ms at 48 kHz */
#define DECIM_OUT_CHUNK  (DECIM_IN_CHUNK / DECIM_M) /* 160 at 8 kHz */

/* Largest block the caller may hand over in one go, i.e. BLOCK_SIZE_MAX/2. */
#define MAX_BLOCK        1920

#define STAGE_LEN        (DECIM_IN_CHUNK + MAX_BLOCK)

/*
 * 8 kHz -> 48 kHz for playback. arm_fir_interpolate_init_f32 requires the tap
 * count to be a multiple of L, so this filter is 96 taps rather than 97.
 */
#define INTERP_L         6
#define INTERP_TAPS      96
#define INTERP_IN_CHUNK  160                             /* 20 ms at 8 kHz */
#define INTERP_OUT_CHUNK (INTERP_IN_CHUNK * INTERP_L)    /* 960 at 48 kHz */
#define OUT_STAGE_LEN    (INTERP_OUT_CHUNK + MAX_BLOCK)

/*
 * Two modem frames is enough: the caller's input ring already absorbs the
 * bursts, so this only has to bridge one demodulator call. Sized for 2400B,
 * whose frames are 1920 samples at 48 kHz rather than the 320 that 1600
 * consumes at 8 kHz.
 */
#define MODEM_FIFO_LEN   4096
#define SPEECH_FIFO_LEN  4096

/*
 * 1600 consumes about 320 modem samples per frame at 8 kHz; 2400B takes 1920
 * at 48 kHz and freedv_nin() swings either side of that as the timing tracker
 * pulls. Sized for the larger case and checked against
 * freedv_get_n_max_modem_samples() at init.
 */
#define MAX_MODEM_SAMPLES 2048

/*
 * Anti-alias lowpass, 97 taps, Blackman-windowed sinc, fc = 3.4 kHz at 48 kHz.
 * Flat through 2.1 kHz so the FDMDV carriers (roughly 900-2100 Hz) are
 * untouched; the band that would alias into that passband after decimation
 * (5.9-7.1 kHz) is down more than 80 dB.
 */
static const float32_t decim_coeffs[DECIM_TAPS] = {
    -5.4097875194e-20f,
    +2.2983367515e-06f,
    +1.0723060970e-05f,
    +2.2990420422e-05f,
    +3.0591254030e-05f,
    +2.1032695066e-05f,
    -1.7344974467e-05f,
    -8.9117606347e-05f,
    -1.8599983701e-04f,
    -2.8382581613e-04f,
    -3.4434044334e-04f,
    -3.2289870361e-04f,
    -1.8156628397e-04f,
    +9.4836903370e-05f,
    +4.8399575363e-04f,
    +9.1790845699e-04f,
    +1.2861459066e-03f,
    +1.4534711728e-03f,
    +1.2912229785e-03f,
    +7.1749533063e-04f,
    -2.6300078098e-04f,
    -1.5311155614e-03f,
    -2.8490849261e-03f,
    -3.8881015821e-03f,
    -4.2890046567e-03f,
    -3.7486499673e-03f,
    -2.1156191226e-03f,
    +5.2763321277e-04f,
    +3.8210181294e-03f,
    +7.1420742491e-03f,
    +9.6904822692e-03f,
    +1.0636958089e-02f,
    +9.3148645018e-03f,
    +5.4183582425e-03f,
    -8.3738829203e-04f,
    -8.6373819887e-03f,
    -1.6601571855e-02f,
    -2.2944891444e-02f,
    -2.5748386333e-02f,
    -2.3301219031e-02f,
    -1.4453181817e-02f,
    +1.0917868063e-03f,
    +2.2605605965e-02f,
    +4.8334902779e-02f,
    +7.5683744961e-02f,
    +1.0155974177e-01f,
    +1.2282954188e-01f,
    +1.3680564963e-01f,
    +1.4167723253e-01f,
    +1.3680564963e-01f,
    +1.2282954188e-01f,
    +1.0155974177e-01f,
    +7.5683744961e-02f,
    +4.8334902779e-02f,
    +2.2605605965e-02f,
    +1.0917868063e-03f,
    -1.4453181817e-02f,
    -2.3301219031e-02f,
    -2.5748386333e-02f,
    -2.2944891444e-02f,
    -1.6601571855e-02f,
    -8.6373819887e-03f,
    -8.3738829203e-04f,
    +5.4183582425e-03f,
    +9.3148645018e-03f,
    +1.0636958089e-02f,
    +9.6904822692e-03f,
    +7.1420742491e-03f,
    +3.8210181294e-03f,
    +5.2763321277e-04f,
    -2.1156191226e-03f,
    -3.7486499673e-03f,
    -4.2890046567e-03f,
    -3.8881015821e-03f,
    -2.8490849261e-03f,
    -1.5311155614e-03f,
    -2.6300078098e-04f,
    +7.1749533063e-04f,
    +1.2912229785e-03f,
    +1.4534711728e-03f,
    +1.2861459066e-03f,
    +9.1790845699e-04f,
    +4.8399575363e-04f,
    +9.4836903370e-05f,
    -1.8156628397e-04f,
    -3.2289870361e-04f,
    -3.4434044334e-04f,
    -2.8382581613e-04f,
    -1.8599983701e-04f,
    -8.9117606347e-05f,
    -1.7344974467e-05f,
    +2.1032695066e-05f,
    +3.0591254030e-05f,
    +2.2990420422e-05f,
    +1.0723060970e-05f,
    +2.2983367515e-06f,
    -5.4097875194e-20f
};

/*
 * Reconstruction lowpass for the 8 kHz -> 48 kHz interpolation, same
 * Blackman-windowed sinc design at fc = 3.4 kHz. DC gain is L rather than 1 to
 * make up for the (L-1) zeros inserted between input samples; the resulting
 * spectral images sit more than 78 dB down.
 */
static const float32_t interp_coeffs[INTERP_TAPS] = {
    -4.1954532168e-19f,
    +1.5588145263e-05f,
    +6.5559848421e-05f,
    +1.2594457973e-04f,
    +1.3850893861e-04f,
    +3.0046835811e-05f,
    -2.5503139247e-04f,
    -7.1760720695e-04f,
    -1.2747942509e-03f,
    -1.7523126280e-03f,
    -1.9086045854e-03f,
    -1.4936756838e-03f,
    -3.3505201417e-04f,
    +1.5682974819e-03f,
    +3.9723237901e-03f,
    +6.3657658008e-03f,
    +8.0247255369e-03f,
    +8.1533067129e-03f,
    +6.0960386939e-03f,
    +1.5814565784e-03f,
    -5.0655510914e-03f,
    -1.2814340642e-02f,
    -1.9984572593e-02f,
    -2.4500455869e-02f,
    -2.4329013022e-02f,
    -1.8033475124e-02f,
    -5.3238946149e-03f,
    +1.2541424584e-02f,
    +3.2645878556e-02f,
    +5.0695596792e-02f,
    +6.1714123028e-02f,
    +6.1075052518e-02f,
    +4.5698908349e-02f,
    +1.5167500247e-02f,
    -2.7516842570e-02f,
    -7.5764873575e-02f,
    -1.1996858339e-01f,
    -1.4880313637e-01f,
    -1.5110045989e-01f,
    -1.1798737264e-01f,
    -4.4887565999e-02f,
    +6.7029061102e-02f,
    +2.1029365172e-01f,
    +3.7167406860e-01f,
    +5.3378679683e-01f,
    +6.7755791645e-01f,
    +7.8512588607e-01f,
    +8.4267378735e-01f,
    +8.4267378735e-01f,
    +7.8512588607e-01f,
    +6.7755791645e-01f,
    +5.3378679683e-01f,
    +3.7167406860e-01f,
    +2.1029365172e-01f,
    +6.7029061102e-02f,
    -4.4887565999e-02f,
    -1.1798737264e-01f,
    -1.5110045989e-01f,
    -1.4880313637e-01f,
    -1.1996858339e-01f,
    -7.5764873575e-02f,
    -2.7516842570e-02f,
    +1.5167500247e-02f,
    +4.5698908349e-02f,
    +6.1075052518e-02f,
    +6.1714123028e-02f,
    +5.0695596792e-02f,
    +3.2645878556e-02f,
    +1.2541424584e-02f,
    -5.3238946149e-03f,
    -1.8033475124e-02f,
    -2.4329013022e-02f,
    -2.4500455869e-02f,
    -1.9984572593e-02f,
    -1.2814340642e-02f,
    -5.0655510914e-03f,
    +1.5814565784e-03f,
    +6.0960386939e-03f,
    +8.1533067129e-03f,
    +8.0247255369e-03f,
    +6.3657658008e-03f,
    +3.9723237901e-03f,
    +1.5682974819e-03f,
    -3.3505201417e-04f,
    -1.4936756838e-03f,
    -1.9086045854e-03f,
    -1.7523126280e-03f,
    -1.2747942509e-03f,
    -7.1760720695e-04f,
    -2.5503139247e-04f,
    +3.0046835811e-05f,
    +1.3850893861e-04f,
    +1.2594457973e-04f,
    +6.5559848421e-05f,
    +1.5588145263e-05f,
    -4.1954532168e-19f
};

static arm_fir_decimate_instance_f32 decim;
static float32_t decim_state[DECIM_TAPS + DECIM_IN_CHUNK - 1];

static float    stage[STAGE_LEN];
static int      stage_used;

static arm_fir_interpolate_instance_f32 interp;
static float32_t interp_state[(INTERP_TAPS / INTERP_L) + INTERP_IN_CHUNK - 1];

static float    out_stage[OUT_STAGE_LEN];
static int      out_stage_used;

/*
 * Playback jitter buffer.
 *
 * For 2400B one modem frame is consumed and one speech frame produced per
 * block, exactly 1:1, so the speech FIFO can never accumulate a cushion on its
 * own. Any frame the deframer misses would then be a permanent hole. Holding
 * playback off until a few frames have banked fixes that: during the wait we
 * produce without consuming, and the cushion carries steady state afterwards.
 *
 * Re-armed on a real underrun so it recovers rather than limping. Sized in
 * frames rather than samples, because 700D produces 1280 speech samples per
 * frame against 320 for the other modes.
 */
static int playback_armed;
static int prefill_samples;

static struct freedv *fdv;
static struct FIFO   *modem_fifo;   /* 8 kHz modem samples awaiting freedv_rx */
static struct FIFO   *speech_fifo;  /* 8 kHz decoded speech */

static int   sync_flag;
static float snr_db;

/*
 * 2400B's modem runs at 48 kHz, the same rate the radio feeds us, so its input
 * goes straight to the demodulator. Only 1600, whose modem wants 8 kHz, needs
 * the decimator. Both return speech at 8 kHz, so the output side is shared.
 */
static int input_is_48k;

int freedv_chain_init(int chain_mode)
{
    /*
     * Re-entrant: switching between 1600 and 2400A means a different modem, so
     * this is called again on a mode change and has to tear the old one down
     * rather than leak it.
     */
    if (fdv != NULL)
    {
        freedv_close(fdv);
        fdv = NULL;
    }
    if (modem_fifo != NULL)
    {
        codec2_fifo_destroy(modem_fifo);
        modem_fifo = NULL;
    }
    if (speech_fifo != NULL)
    {
        codec2_fifo_destroy(speech_fifo);
        speech_fifo = NULL;
    }

    stage_used     = 0;
    out_stage_used = 0;
    playback_armed = 0;
    sync_flag      = 0;
    snr_db         = 0.0f;

    if (arm_fir_decimate_init_f32(&decim, DECIM_TAPS, DECIM_M,
                                  (float32_t *)decim_coeffs, decim_state,
                                  DECIM_IN_CHUNK) != ARM_MATH_SUCCESS)
        return -1;

    if (arm_fir_interpolate_init_f32(&interp, INTERP_L, INTERP_TAPS,
                                     (float32_t *)interp_coeffs, interp_state,
                                     INTERP_IN_CHUNK) != ARM_MATH_SUCCESS)
        return -1;

    input_is_48k = (chain_mode == FREEDV_CHAIN_MODE_2400B);

    int m = FREEDV_MODE_1600;
    if (chain_mode == FREEDV_CHAIN_MODE_2400B) m = FREEDV_MODE_2400B;
    if (chain_mode == FREEDV_CHAIN_MODE_700D)  m = FREEDV_MODE_700D;
    if (chain_mode == FREEDV_CHAIN_MODE_700E)  m = FREEDV_MODE_700E;

    fdv = freedv_open(m);
    if (fdv == NULL)
        return -1;

    if (freedv_get_n_max_modem_samples(fdv) > MAX_MODEM_SAMPLES)
        return -1;

    prefill_samples = 3 * freedv_get_n_speech_samples(fdv);
    if (prefill_samples > SPEECH_FIFO_LEN / 2)
        prefill_samples = SPEECH_FIFO_LEN / 2;

    /*
     * Squelch on, which here means "output nothing until synced".
     *
     * With it off, freedv_bits_to_speech() passes the modem input through
     * while unsynced by taking every rate_factor'th sample with no anti-alias
     * filter at all. At 48 kHz in and 8 kHz out that folds everything above
     * 4 kHz back into the speech band, and the result played out sounds like
     * badly resampled audio rather than the silence a radio should give.
     */
    freedv_set_squelch_en(fdv, true);

    modem_fifo  = codec2_fifo_create(MODEM_FIFO_LEN);
    speech_fifo = codec2_fifo_create(SPEECH_FIFO_LEN);
    if (modem_fifo == NULL || speech_fifo == NULL)
        return -1;

    return 0;
}

/*
 * Demodulate at most MAX_FRAMES_PER_CALL frames.
 *
 * One 20 ms audio block yields 160 samples at 8 kHz while a FreeDV 1600 frame
 * consumes about 320, so a frame only comes due every second block. Draining
 * the whole FIFO in one go therefore makes the per-block cost alternate
 * between cheap and expensive, and the expensive one is what overruns the
 * block period. Capping the work per call flattens that out; the FIFO carries
 * the remainder, and one frame per block is still twice the rate needed to
 * keep up, so any backlog drains quickly.
 */
#define MAX_FRAMES_PER_CALL 1

static void run_demod(void)
{
    /*
     * Static rather than automatic: this runs off the back of the audio
     * callback, and fdmdv_demod() already puts sizeable VLAs on the stack.
     */
    static short demod_in[MAX_MODEM_SAMPLES];
    static short speech[MAX_MODEM_SAMPLES];

    for (int frame = 0; frame < MAX_FRAMES_PER_CALL; frame++)
    {
        int nin = freedv_nin(fdv);

        if (nin > MAX_MODEM_SAMPLES)
            return;                              /* guarded at init */
        if (codec2_fifo_used(modem_fifo) < nin)
            return;

        codec2_fifo_read(modem_fifo, demod_in, nin);

        int nout = freedv_rx(fdv, speech, demod_in);

        freedv_get_modem_stats(fdv, &sync_flag, &snr_db);

        if (nout > 0 && codec2_fifo_free(speech_fifo) >= nout)
            codec2_fifo_write(speech_fifo, speech, nout);
    }
}

void freedv_chain_put_audio48(const float *audio48, int n)
{
    if (fdv == NULL)
        return;

    if (input_is_48k)
    {
        /* 2400B's modem is already at 48 kHz: straight through, no resampling. */
        while (n > 0)
        {
            static short out16[256];

            int take = (n < (int)(sizeof(out16) / sizeof(out16[0])))
                     ? n : (int)(sizeof(out16) / sizeof(out16[0]));

            for (int i = 0; i < take; i++)
            {
                float v = audio48[i] * 16384.0f;

                if (v >  32767.0f) v =  32767.0f;
                if (v < -32768.0f) v = -32768.0f;

                out16[i] = (short)v;
            }

            if (codec2_fifo_free(modem_fifo) >= take)
                codec2_fifo_write(modem_fifo, out16, take);

            audio48 += take;
            n       -= take;
        }

        run_demod();
        return;
    }

    while (n > 0)
    {
        int space = STAGE_LEN - stage_used;
        int take  = (n < space) ? n : space;

        memcpy(&stage[stage_used], audio48, take * sizeof(float));
        stage_used += take;
        audio48    += take;
        n          -= take;

        while (stage_used >= DECIM_IN_CHUNK)
        {
            float32_t out8[DECIM_OUT_CHUNK];
            short     out16[DECIM_OUT_CHUNK];

            arm_fir_decimate_f32(&decim, stage, out8, DECIM_IN_CHUNK);

            for (int i = 0; i < DECIM_OUT_CHUNK; i++)
            {
                float v = out8[i] * 16384.0f;

                if (v >  32767.0f) v =  32767.0f;
                if (v < -32768.0f) v = -32768.0f;

                out16[i] = (short)v;
            }

            if (codec2_fifo_free(modem_fifo) >= DECIM_OUT_CHUNK)
                codec2_fifo_write(modem_fifo, out16, DECIM_OUT_CHUNK);

            stage_used -= DECIM_IN_CHUNK;
            memmove(stage, &stage[DECIM_IN_CHUNK], stage_used * sizeof(float));

            run_demod();
        }
    }
}

int freedv_chain_get_speech8(int16_t *speech_out, int max)
{
    if (fdv == NULL)
        return 0;

    int avail = codec2_fifo_used(speech_fifo);
    int n     = (avail < max) ? avail : max;

    if (n > 0)
        codec2_fifo_read(speech_fifo, (short *)speech_out, n);

    return n;
}

int freedv_chain_get_speech48(float *audio48, int n)
{
    int produced = 0;

    if (fdv == NULL)
    {
        memset(audio48, 0, n * sizeof(float));
        return 0;
    }

    /* Let a cushion bank up before playing anything; see PREFILL_SAMPLES. */
    if (!playback_armed)
    {
        if (out_stage_used + codec2_fifo_used(speech_fifo) < prefill_samples)
        {
            memset(audio48, 0, n * sizeof(float));
            return 0;
        }
        playback_armed = 1;
    }

    /* Top the 48 kHz staging buffer up while whole 8 kHz chunks are available. */
    while (out_stage_used < n &&
           out_stage_used + INTERP_OUT_CHUNK <= OUT_STAGE_LEN &&
           codec2_fifo_used(speech_fifo) >= INTERP_IN_CHUNK)
    {
        static short     in16[INTERP_IN_CHUNK];
        static float32_t in8[INTERP_IN_CHUNK];

        codec2_fifo_read(speech_fifo, in16, INTERP_IN_CHUNK);

        /* Back to the normalised +-1.0 convention the SSB path uses. */
        for (int i = 0; i < INTERP_IN_CHUNK; i++)
            in8[i] = (float)in16[i] / 32768.0f;

        arm_fir_interpolate_f32(&interp, in8, &out_stage[out_stage_used],
                                INTERP_IN_CHUNK);

        out_stage_used += INTERP_OUT_CHUNK;
    }

    produced = (out_stage_used < n) ? out_stage_used : n;

    if (produced > 0)
    {
        memcpy(audio48, out_stage, produced * sizeof(float));

        out_stage_used -= produced;
        memmove(out_stage, &out_stage[produced], out_stage_used * sizeof(float));
    }

    /* Underrun (no sync yet, or waiting on the next frame) plays as silence. */
    if (produced < n)
    {
        memset(&audio48[produced], 0, (n - produced) * sizeof(float));

        /* Ran dry: rebuild the cushion instead of gapping every block. */
        playback_armed = 0;
    }

    return produced;
}

void freedv_chain_reset(void)
{
    if (fdv == NULL)
        return;

    /*
     * Called when the radio switches modes. The resamplers and FIFOs hold
     * audio from before the switch, and the block size may have changed under
     * them, so everything buffered is discarded and the modem is told to
     * re-acquire rather than carry stale timing into the new stream.
     */
    stage_used     = 0;
    out_stage_used = 0;
    playback_armed = 0;
    sync_flag      = 0;
    snr_db         = 0.0f;

    memset(decim_state,  0, sizeof(decim_state));
    memset(interp_state, 0, sizeof(interp_state));

    codec2_fifo_destroy(modem_fifo);
    codec2_fifo_destroy(speech_fifo);

    modem_fifo  = codec2_fifo_create(MODEM_FIFO_LEN);
    speech_fifo = codec2_fifo_create(SPEECH_FIFO_LEN);
}

int freedv_chain_synced(void)
{
    return sync_flag;
}

float freedv_chain_snr(void)
{
    return snr_db;
}
