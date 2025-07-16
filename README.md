# Advanced Computer Graphics

This course explored modern rendering techniques through three progressive assignments: BVH acceleration for optimized ray tracing, radiosity methods for global illumination with Monte Carlo integration, and unbiased path tracing

## Assignment 1: Accelerated Ray Tracing

This assignment focused on optimizing a naive ray tracer through acceleration structures and advanced rendering techniques. It included implementing BVH construction and traversal algorithms to dramatically reduce ray-scene intersection costs (R1), adding BVH saving and loading functionality for persistent acceleration structures (R2), incorporating simple texturing to enhance visual appearance (R3), implementing ambient occlusion for realistic global illumination approximation (R4), and adding multithreading support for parallel ray processing (R5).

## Assignment 2: Radiosity

This assignment focused on implementing global illumination through radiosity methods using the previously developed accelerated ray tracer. It included integrating the BVH-accelerated ray tracing code from Assignment 1 as the foundation (R1), implementing area light sources with proper solid-angle-to-area variable changes for realistic direct lighting (R2), and developing a complete radiosity system using Monte Carlo integration with importance sampling and probability density functions to compute diffuse global illumination (R3).

## Assignment 3: Path Tracing

This assignment focused on implementing a Monte Carlo path tracer for unbiased global illumination. It included integrating the existing accelerated ray tracer foundation from previous assignments (R1), implementing direct lighting with proper shadow calculations (R2), developing bounced indirect lighting through recursive path tracing for realistic global illumination (R3), and incorporating Russian roulette termination for unbiased rendering with finite computation (R4).
