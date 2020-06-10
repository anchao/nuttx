OPUS_DIR += $(BUILD_DIR)/components/thirdparty/opus

OPUS_LIBRARY_FILES=		$(OPUS_DIR)/celt/bands.o \
                        $(OPUS_DIR)/celt/celt.o \
                        $(OPUS_DIR)/celt/celt_encoder.o \
                        $(OPUS_DIR)/celt/celt_decoder.o \
                        $(OPUS_DIR)/celt/cwrs.o  \
                        $(OPUS_DIR)/celt/entcode.o \
                        $(OPUS_DIR)/celt/entdec.o   \
                        $(OPUS_DIR)/celt/entenc.o   \
                        $(OPUS_DIR)/celt/kiss_fft.o  \
                        $(OPUS_DIR)/celt/laplace.o  \
                        $(OPUS_DIR)/celt/mathops.o  \
                        $(OPUS_DIR)/celt/mdct.o   \
                        $(OPUS_DIR)/celt/modes.o   \
                        $(OPUS_DIR)/celt/pitch.o   \
                        $(OPUS_DIR)/celt/celt_lpc.o \
                        $(OPUS_DIR)/celt/quant_bands.o \
                        $(OPUS_DIR)/celt/rate.o  \
                        $(OPUS_DIR)/celt/vq.o  \
                        $(OPUS_DIR)/celt/arm/armcpu.o \
                        $(OPUS_DIR)/celt/arm/arm_celt_map.o  \
                        $(OPUS_DIR)/celt/arm/celt_neon_intr.o  \
                        $(OPUS_DIR)/celt/arm/pitch_neon_intr.o  \
                        $(OPUS_DIR)/silk/CNG.o  \
                        $(OPUS_DIR)/silk/code_signs.o  \
                        $(OPUS_DIR)/silk/init_decoder.o  \
                        $(OPUS_DIR)/silk/decode_core.o  \
                        $(OPUS_DIR)/silk/decode_frame.o  \
                        $(OPUS_DIR)/silk/decode_parameters.o \
                        $(OPUS_DIR)/silk/decode_indices.o  \
                        $(OPUS_DIR)/silk/decode_pulses.o  \
                        $(OPUS_DIR)/silk/decoder_set_fs.o  \
                        $(OPUS_DIR)/silk/dec_API.o  \
                        $(OPUS_DIR)/silk/enc_API.o   \
                        $(OPUS_DIR)/silk/encode_indices.o  \
                        $(OPUS_DIR)/silk/encode_pulses.o  \
                        $(OPUS_DIR)/silk/gain_quant.o  \
                        $(OPUS_DIR)/silk/interpolate.o  \
                        $(OPUS_DIR)/silk/LP_variable_cutoff.o \
                        $(OPUS_DIR)/silk/NLSF_decode.o  \
                        $(OPUS_DIR)/silk/NSQ.o   \
                        $(OPUS_DIR)/silk/NSQ_del_dec.o \
                        $(OPUS_DIR)/silk/PLC.o  \
                        $(OPUS_DIR)/silk/shell_coder.o \
                        $(OPUS_DIR)/silk/tables_gain.o  \
                        $(OPUS_DIR)/silk/tables_LTP.o   \
                        $(OPUS_DIR)/silk/tables_NLSF_CB_NB_MB.o \
                        $(OPUS_DIR)/silk/tables_NLSF_CB_WB.o  \
                        $(OPUS_DIR)/silk/tables_other.o  \
                        $(OPUS_DIR)/silk/tables_pitch_lag.o  \
                        $(OPUS_DIR)/silk/tables_pulses_per_block.o \
                        $(OPUS_DIR)/silk/VAD.o  \
                        $(OPUS_DIR)/silk/control_audio_bandwidth.o \
                        $(OPUS_DIR)/silk/quant_LTP_gains.o   \
                        $(OPUS_DIR)/silk/VQ_WMat_EC.o   \
                        $(OPUS_DIR)/silk/HP_variable_cutoff.o \
                        $(OPUS_DIR)/silk/NLSF_encode.o   \
                        $(OPUS_DIR)/silk/NLSF_VQ.o   \
                        $(OPUS_DIR)/silk/NLSF_unpack.o \
                        $(OPUS_DIR)/silk/NLSF_del_dec_quant.o  \
                        $(OPUS_DIR)/silk/process_NLSFs.o  \
                        $(OPUS_DIR)/silk/stereo_LR_to_MS.o \
                        $(OPUS_DIR)/silk/stereo_MS_to_LR.o  \
                        $(OPUS_DIR)/silk/check_control_input.o  \
                        $(OPUS_DIR)/silk/control_SNR.o  \
                        $(OPUS_DIR)/silk/init_encoder.o  \
                        $(OPUS_DIR)/silk/control_codec.o  \
                        $(OPUS_DIR)/silk/A2NLSF.o  \
                        $(OPUS_DIR)/silk/ana_filt_bank_1.o \
                        $(OPUS_DIR)/silk/biquad_alt.o  \
                        $(OPUS_DIR)/silk/bwexpander_32.o  \
                        $(OPUS_DIR)/silk/bwexpander.o  \
                        $(OPUS_DIR)/silk/debug.o  \
                        $(OPUS_DIR)/silk/decode_pitch.o  \
                        $(OPUS_DIR)/silk/inner_prod_aligned.o  \
                        $(OPUS_DIR)/silk/lin2log.o \
                        $(OPUS_DIR)/silk/log2lin.o  \
                        $(OPUS_DIR)/silk/LPC_analysis_filter.o \
                        $(OPUS_DIR)/silk/LPC_inv_pred_gain.o  \
                        $(OPUS_DIR)/silk/table_LSF_cos.o  \
                        $(OPUS_DIR)/silk/NLSF2A.o   \
                        $(OPUS_DIR)/silk/NLSF_stabilize.o  \
                        $(OPUS_DIR)/silk/NLSF_VQ_weights_laroia.o \
                        $(OPUS_DIR)/silk/pitch_est_tables.o \
                        $(OPUS_DIR)/silk/resampler.o  \
                        $(OPUS_DIR)/silk/resampler_down2_3.o  \
                        $(OPUS_DIR)/silk/resampler_down2.o  \
                        $(OPUS_DIR)/silk/resampler_private_AR2.o \
                        $(OPUS_DIR)/silk/resampler_private_down_FIR.o \
                        $(OPUS_DIR)/silk/resampler_private_IIR_FIR.o  \
                        $(OPUS_DIR)/silk/resampler_private_up2_HQ.o   \
                        $(OPUS_DIR)/silk/resampler_rom.o  \
                        $(OPUS_DIR)/silk/sigm_Q15.o  \
                        $(OPUS_DIR)/silk/sort.o  \
                        $(OPUS_DIR)/silk/sum_sqr_shift.o \
                        $(OPUS_DIR)/silk/stereo_decode_pred.o \
                        $(OPUS_DIR)/silk/stereo_encode_pred.o  \
                        $(OPUS_DIR)/silk/stereo_find_predictor.o \
                        $(OPUS_DIR)/silk/stereo_quant_pred.o  \
                        $(OPUS_DIR)/silk/LPC_fit.o  \
                        $(OPUS_DIR)/silk/fixed/LTP_analysis_filter_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/LTP_scale_ctrl_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/corrMatrix_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/encode_frame_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/find_LPC_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/find_LTP_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/find_pitch_lags_FIX.o \
                        $(OPUS_DIR)/silk/fixed/find_pred_coefs_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/noise_shape_analysis_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/process_gains_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/regularize_correlations_FIX.o \
                        $(OPUS_DIR)/silk/fixed/residual_energy16_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/residual_energy_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/warped_autocorrelation_FIX.o \
                        $(OPUS_DIR)/silk/fixed/apply_sine_window_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/autocorr_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/burg_modified_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/k2a_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/k2a_Q16_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/pitch_analysis_core_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/vector_ops_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/schur64_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/schur_FIX.o  \
                        $(OPUS_DIR)/silk/fixed/arm/warped_autocorrelation_FIX_neon_intr.o  \
                        $(OPUS_DIR)/silk/arm/arm_silk_map.o   \
                        $(OPUS_DIR)/silk/arm/biquad_alt_neon_intr.o  \
                        $(OPUS_DIR)/silk/arm/LPC_inv_pred_gain_neon_intr.o  \
                        $(OPUS_DIR)/silk/arm/NSQ_del_dec_neon_intr.o   \
                        $(OPUS_DIR)/silk/arm/NSQ_neon.o  \
                        $(OPUS_DIR)/src/opus.o  \
                        $(OPUS_DIR)/src/opus_decoder.o \
                        $(OPUS_DIR)/src/opus_encoder.o  \
                        $(OPUS_DIR)/src/opus_multistream.o  \
                        $(OPUS_DIR)/src/opus_multistream_encoder.o \
                        $(OPUS_DIR)/src/opus_multistream_decoder.o  \
                        $(OPUS_DIR)/src/repacketizer.o   \
                        $(OPUS_DIR)/src/opus_projection_encoder.o  \
                        $(OPUS_DIR)/src/opus_projection_decoder.o  \
                        $(OPUS_DIR)/src/mapping_matrix.o \
						$(OPUS_DIR)/src/analysis.o \
						$(OPUS_DIR)/src/mlp.o \
						$(OPUS_DIR)/src/mlp_data.o \
						$(OPUS_DIR)/celt/arm/celt_pitch_xcorr_arm.o

OPUS_PROGRAMS_FILES-$(CONFIG_COMPONENTS_OPUS_TEST)= $(OPUS_DIR)/doc/trivial_example.o \
		$(OPUS_DIR)/tests/opus_encode_regressions.o \
		$(OPUS_DIR)/tests/test_opus_api.o \
		$(OPUS_DIR)/tests/test_opus_decode.o \
		$(OPUS_DIR)/tests/test_opus_encode.o \
		$(OPUS_DIR)/tests/test_opus_padding.o \
		$(OPUS_DIR)/tests/test_opus_projection.o

COMPONENTS_OPUS_OBJECTS=$(OPUS_LIBRARY_FILES)  \
			         $(OPUS_PROGRAMS_FILES-y)

$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/include/FreeRTOS_POSIX/
$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/components/thirdparty/opus/
$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/components/thirdparty/opus/include
$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/components/thirdparty/opus/celt
$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/components/thirdparty/opus/silk
$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/components/thirdparty/opus/silk/float
$(COMPONENTS_OPUS_OBJECTS):CFLAGS += -I $(BASE)/components/thirdparty/opus/silk/fixed

CC_FLAGS += -DHAVE_CONFIG_H

CC_FLAGS += -DFIXED_POINT=1
#CC_FLAGS += -DDISABLE_FLOAT_API=1
AS_FLAGS += -DOPUS_ARM_ASM

AS_FLAGS += -DOPUS_ARM_INLINE_ASM=1
#AS_FLAGS += -DOPUS_ARM_INLINE_EDSP=1
AS_FLAGS += -DOPUS_ARM_INLINE_MEDIA=1
AS_FLAGS += -DOPUS_ARM_INLINE_NEON=1
#AS_FLAGS += -DOPUS_ARM_MAY_HAVE_EDSP=1
AS_FLAGS += -DOPUS_ARM_MAY_HAVE_MEDIA=1
AS_FLAGS += -DOPUS_ARM_MAY_HAVE_NEON=1

#AS_FLAGS += -DOPUS_ARM_PRESUME_EDSP=1
AS_FLAGS += -DOPUS_ARM_PRESUME_MEDIA=1
AS_FLAGS += -DOPUS_ARM_PRESUME_NEON=1

$(COMPONENTS_OPUS_OBJECTS):CFLAGS +=  $(CC_FLAGS)
$(COMPONENTS_OPUS_OBJECTS):ASFLAGS +=  $(AS_FLAGS)

$(COMPONENTS_OPUS_OBJECTS):MODULE_NAME="components-OPUS"

OBJECTS += $(COMPONENTS_OPUS_OBJECTS)
