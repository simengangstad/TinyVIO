# Fetches the dependencies for the project and sets up the flags for the respective 
# dependencies

include(FetchContent)

# cmake_policy(SET CMP0135 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

set(FETCHCONTENT_QUIET FALSE)

# Fetch ETL
FetchContent_Declare(
    etl
    GIT_REPOSITORY https://github.com/ETLCPP/etl
    GIT_TAG 20.35.14
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)

# Fetch Eigen
set(EIGEN_BUILD_DOC OFF)
set(BUILD_TESTING OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)

FetchContent_Declare(
  Eigen
  GIT_REPOSITORY https://github.com/simengangstad/Eigen.git
  GIT_TAG origin/3.4.0-custom-allocator 
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(etl Eigen)

set(EIGEN_FLAGS
    EIGEN_NO_CUDA
    EIGEN_DONT_VECTORIZE
    EIGEN_MALLOC_ALREADY_ALIGNED=0
    EIGEN_MAX_ALIGN_BYTES=16
    EIGEN_STACK_ALLOCATION_LIMIT=16384
    EIGEN_DEFAULT_L1_CACHE_SIZE=32768
    EIGEN_USE_CUSTOM_ALLOCATOR
)

set(ETL_FLAGS
    $<$<CONFIG:Debug>:ETL_LOG_ERRORS>
    $<$<CONFIG:Debug>:ETL_VERBOSE_ERRORS>
    $<$<CONFIG:Debug>:ETL_CHECK_PUSH_POP>
)

