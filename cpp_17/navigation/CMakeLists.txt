add_library(
    iasdk_navigation STATIC 
    peak_finder.cpp
    sector_blanking.cpp
)

target_link_libraries(iasdk_navigation iasdk_utility)
