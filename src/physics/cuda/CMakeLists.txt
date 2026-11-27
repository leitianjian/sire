add_library(sire_cuda STATIC
	contact_trajectory.cu
	contact_history.cu
	flat_ground_collision.cu
	rigid_body_contact.cu
	rigid_body_control.cu
	rigid_body_dynamics.cu
	rigid_body_events.cu
	rigid_body_integration.cu
	rigid_body_joint_state.cu
	rigid_body_limits.cu
	rigid_body_reset.cu
	spectral_admm.cu)
set_target_properties(sire_cuda PROPERTIES
	CUDA_STANDARD 17
	CUDA_STANDARD_REQUIRED ON
	POSITION_INDEPENDENT_CODE ON)
target_include_directories(sire_cuda
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
	PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(sire_cuda PUBLIC CUDA::cudart_static)

# Keep the COAL port's support and simplex tie decisions close to the host
# build, which evaluates the same expressions without fused operations.
set_property(SOURCE flat_ground_collision.cu APPEND PROPERTY COMPILE_OPTIONS
	"$<$<CUDA_COMPILER_ID:NVIDIA>:--fmad=false>"
	"$<$<CUDA_COMPILER_ID:NVIDIA>:--expt-relaxed-constexpr>"
	"$<$<CUDA_COMPILER_ID:Clang>:-ffp-contract=off>")
