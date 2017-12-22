
#ifndef MANIPULATION_PLANNING_SUITE_FMM_H
#define MANIPULATION_PLANNING_SUITE_FMM_H

#include <vector>
#include <stdexcept>
#include <iostream>
#include <math.h>
#include <sim_env/Grid.h>

// #include <boost/heap/fibonacci_heap.cpp>
#include <boost/heap/binomial_heap.hpp>

namespace mps {
    namespace sdf {
        namespace fmm {
            // Returns true if both grids have same dimensions, else false
            template<typename T_1, typename T_2>
            bool compareGridSize(const sim_env::grid::Grid3D<T_1>& grid_1, const sim_env::grid::Grid3D<T_2>& grid_2) {
                if (grid_1.getXSize() != grid_2.getXSize() ||
                    grid_1.getYSize() != grid_2.getYSize() ||
                    grid_1.getZSize() != grid_2.getZSize())
                {
                    return false;
                }
                return true;
            }

            // State of voxels in the fmm algorithm
            enum VoxelState {Frozen, Considered, Far};
            template<typename ScalarType>
            struct PriorityQueueElement {
                ScalarType value;
                sim_env::grid::UnsignedIndex index;
                PriorityQueueElement(const ScalarType& val, const sim_env::grid::UnsignedIndex& idx) : value(val), index(idx) {}
                PriorityQueueElement() = default;
                PriorityQueueElement(const PriorityQueueElement& other) = default;
                PriorityQueueElement& operator=(const PriorityQueueElement& other) = default;
                ~PriorityQueueElement() = default;
            };

            template<typename ScalarType>
            struct PriorityQueueElementComparator {
                inline bool operator()(const PriorityQueueElement<ScalarType>& a, const PriorityQueueElement<ScalarType>& b) const {
                    return a.value >= b.value;
                }
            };

            // Data stored for each voxel in fmm
            template<typename ScalarType, typename HandleType>
            struct FMMVoxelData {
                ScalarType distance;
                int sign;
                VoxelState state;
                HandleType pq_handle;
            };

            /**
             * Black board data structure for fmm algorithm.
             */
            template<typename ScalarType>
            struct FMMData {
                typedef boost::heap::binomial_heap<PriorityQueueElement<ScalarType> , boost::heap::stable<false>, boost::heap::compare<PriorityQueueElementComparator<ScalarType> > > PriorityQueue;
                typedef FMMVoxelData<ScalarType, typename PriorityQueue::handle_type> TypedFMMVoxelData;
                PriorityQueue considered_voxels;
                const sim_env::grid::Grid3D<int>& input_grid;
                sim_env::grid::Grid3D<TypedFMMVoxelData> grid;
                FMMData(const sim_env::grid::Grid3D<int>& igrid) :
                    input_grid(igrid),
                    grid(igrid.getXSize(), igrid.getYSize(), igrid.getZSize()) {}
                FMMData(const FMMData& other) = default;
                ~FMMData() = default;

                void copySignedDistances(sim_env::grid::Grid3D<ScalarType>& output, ScalarType scale=1.0) {
                    if (not compareGridSize<ScalarType, TypedFMMVoxelData>(output, grid)) {
                        throw std::logic_error("Could not copy signed distance. Given output grid has invalid dimensions.");
                    }
                    auto idx_generator = grid.getIndexGenerator();
                    while (idx_generator.hasNext()) {
                        auto idx = idx_generator.next();
                        output(idx) = scale * grid(idx).sign * grid(idx).distance;
                    }
                }
            };


            /**
             * Compute the real roots of a polynomial of degree 2 (i.e. quadratic).
             * @param coeffs - an array of length 3 containing the coefficients {c, b, a} for ax^2 + bx + c = 0
             * @param roots - an array of length 2, which will contain the solutions
             * @return number of real roots i.e. either 0, 1 or 2
             */
            template<typename ScalarType>
            int findQuadraticRoots(const std::array<ScalarType, 3>& coeffs, std::array<ScalarType, 2>& roots) {
                roots[0] = 0.0;
                roots[1] = 0.0;
                if (coeffs[2] == 0.0) { // linear polygon
                    if (coeffs[1] != 0.0) {
                        roots[0] = -coeffs[0] / coeffs[1];
                        return 1;
                    }
                    return 0; // no solution (or infinite if c == 0)
                }
                // we have a polygon of degree 2
                ScalarType root_arg = coeffs[1] * coeffs[1] - 4.0 * coeffs[0] * coeffs[2];
                if (root_arg < 0.0) {
                    return 0;
                } else if (root_arg == 0.0) {
                    roots[0] = -coeffs[1] / (ScalarType(2) * coeffs[2]);
                    return 1;
                }
                roots[0] = (-coeffs[1] - sqrt(root_arg)) / (ScalarType(2) * coeffs[2]);
                roots[1] = (-coeffs[1] + sqrt(root_arg)) / (ScalarType(2) * coeffs[2]);
                return 2;
            }

            template<typename ScalarType>
            void computeCoefficients(const sim_env::grid::UnsignedIndex& idx,
                                     std::array<ScalarType, 3>& coeffs,
                                     FMMData<ScalarType>& fmm_data) {
                std::array<sim_env::grid::SignedIndex, 6> directions = {{sim_env::grid::SignedIndex(-1, 0, 0),
                                                               sim_env::grid::SignedIndex(1, 0, 0),
                                                               sim_env::grid::SignedIndex(0, -1, 0),
                                                               sim_env::grid::SignedIndex(0, 1, 0),
                                                               sim_env::grid::SignedIndex(0, 0, -1),
                                                               sim_env::grid::SignedIndex(0, 0, 1)}};
                sim_env::grid::SignedIndex center_idx(idx.ix, idx.iy, idx.iz);
                for (unsigned int r = 0; r < 3; ++r) { // for each axis x, y, z
                    // we need to figure whether we take forward or backward gradient
                    // as well as whether to use first order or second order derivative
                    ScalarType val_1(std::numeric_limits<ScalarType>::max());
                    ScalarType val_2(std::numeric_limits<ScalarType>::max());
                    for (unsigned int c = 0; c < 2; ++c) { // for forward and backward direction
                        auto d1_idx = center_idx + directions[r * 2 + c]; // idx - 1
                        if (fmm_data.grid.inBounds(d1_idx) and
                            fmm_data.grid(d1_idx.toUnsignedIndex()).state == VoxelState::Frozen) {
                            // idx - 1 is valid, so store value
                            ScalarType tval_1 = fmm_data.grid(d1_idx.toUnsignedIndex()).distance;
                            if (tval_1 < val_1) { // if it is smaller than for the other direction
                                val_1 = tval_1;
                                // let's see whether we can get a second order derivative
                                auto d2_idx = center_idx + directions[r * 2 + c] * 2;
                                if (fmm_data.grid.inBounds(d2_idx) and
                                    fmm_data.grid(d2_idx.toUnsignedIndex()).state == VoxelState::Frozen) {
                                    ScalarType tval_2 = fmm_data.grid(d2_idx.toUnsignedIndex()).distance;
                                    if (tval_2 < tval_1) {
                                        val_2 = tval_2;
                                    } else {
                                        val_2 = std::numeric_limits<ScalarType>::max();
                                    }
                                } else {
                                    val_2 = std::numeric_limits<ScalarType>::max();
                                }
                            }
                        }
                    }
                    // in case we can use the second order derivative
                    if (val_2 != std::numeric_limits<ScalarType>::max()) {
                        ScalarType k = ScalarType(1) / ScalarType(3) * (ScalarType(4) * val_1 - val_2);
                        ScalarType a = ScalarType(9) / ScalarType(4);
                        coeffs[2] += a;
                        coeffs[1] -= ScalarType(2) * a * k;
                        coeffs[0] += a * k * k;
                    } else if (val_1 != std::numeric_limits<ScalarType>::max()) {  // first order derivative
                        coeffs[2] += ScalarType(1);
                        coeffs[1] -= ScalarType(2) * val_1;
                        coeffs[0] += val_1 * val_1;
                    }
                }
            }

            /**
             * Computes the minimal distance to the intial condition for the voxel with index idx.
             * For the distance computation only the frozen neighbors are taken into account. Thus, if
             * idx has no frozen neighbor, the returned distance if infinity.
             */
            template<typename ScalarType>
            ScalarType computeDistance(const sim_env::grid::UnsignedIndex& idx, FMMData<ScalarType>& fmm_data) {
                std::array<ScalarType, 3> coeffs = {{-1.0, 0.0, 0.0}};
                std::array<ScalarType, 2> roots = {{0.0, 0.0}};
                computeCoefficients<ScalarType>(idx, coeffs, fmm_data);
                int num_solutions = findQuadraticRoots<ScalarType>(coeffs, roots);
                ScalarType new_distance = 0.0;
                switch(num_solutions)
                {
                    case 2:
                        new_distance = std::max(roots[0], roots[1]);
                        break;
                    case 1:
                        new_distance = roots[0];
                        break;
                    case 0:
                        new_distance = std::numeric_limits<ScalarType>::max();
                        break;
                }
                return new_distance;
            }

            /**
             * Computes and updates the distance of the voxel at location idx.
             */
            template<typename ScalarType>
            void updateDistance(const sim_env::grid::UnsignedIndex& idx, FMMData<ScalarType>& fmm_data) {
                if (fmm_data.grid(idx).state == VoxelState::Frozen) {
                    return;
                }
                ScalarType dist = computeDistance<ScalarType>(idx, fmm_data);
                if (fmm_data.grid(idx).state == VoxelState::Considered) {
                    fmm_data.considered_voxels.decrease(fmm_data.grid(idx).pq_handle, PriorityQueueElement<ScalarType>(dist, idx));
                    fmm_data.grid(idx).distance = dist;
                } else if (dist != std::numeric_limits<ScalarType>::max()) {
                    assert(fmm_data.grid(idx).state == VoxelState::Far);
                    fmm_data.grid(idx).state = VoxelState::Considered;
                    fmm_data.grid(idx).pq_handle = fmm_data.considered_voxels.emplace(PriorityQueueElement<ScalarType>(dist, idx));
                    fmm_data.grid(idx).distance = dist;
                }
            }

            /**
             * Initializes the provided fmm_data for FMM.
             * @param fmm_data - FMMData structure to initialize
             * @param frozen_states - the states that form the initial condition, i.e. boundary, are returned
             *                      in this list.
             */
            template<typename ScalarType>
            void initializeData(FMMData<ScalarType>& fmm_data, std::vector<sim_env::grid::UnsignedIndex>& frozen_states) {
                // Initialize data structure
                auto idx_generator = fmm_data.input_grid.getIndexGenerator();
                // Run over all voxels
                while (idx_generator.hasNext()) {
                    auto index = idx_generator.next();
                    fmm_data.grid(index).sign = fmm_data.input_grid(index);
                    fmm_data.grid(index).state = VoxelState::Far;
                    // check if this index is on the boundary
                    // it has to have positive sign for that and have at least one neigbor with negative sign
                    if (fmm_data.input_grid(index) == 1) {
                        auto neighbor_gen = fmm_data.input_grid.getNeighborIndexGenerator(index, 1, 1, 1);
                        bool on_boundary = false;
                        while (neighbor_gen.hasNext() and not on_boundary) {
                            auto neighbor_idx = neighbor_gen.next();
                            on_boundary = fmm_data.input_grid(neighbor_idx) == -1;
                        }
                        if (on_boundary) {
                            fmm_data.grid(index).state = VoxelState::Frozen;
                            fmm_data.grid(index).distance = 0.0;
                            frozen_states.push_back(index);
                        }
                    }
                }
            }

            /**
             * Computes a signed distance field using the fast marching method.
             * @param input_grid - a grid of integers filled with 1 and -1.
             *                     1 for points outside and -1 for points inside of the boundary.
             *                     The points x for which input_grid(x) == 1 and there is an immediate neighbor of
             *                     x for which input_grid(x) == -1, constitute the boundary
             * @param result_grid - a grid that will after temination of this algorithm
             *                  contain the approximate signed closest distance to the boundary.
             * @param scale - a factor translating voxel size to a desired untit (i.e. m)
             */
            template<typename ScalarType>
            void computeSDF(const sim_env::grid::Grid3D<int>& input_grid,
                            sim_env::grid::Grid3D<ScalarType>& result_grid,
                            ScalarType scale=ScalarType(1))
            {
                if (not compareGridSize(input_grid, result_grid)) {
                    throw std::logic_error("Input and output grid do not have the same dimension.");
                }
                FMMData<ScalarType> fmm_data(input_grid);
                std::vector<sim_env::grid::UnsignedIndex> frozen_states;
                initializeData(fmm_data, frozen_states);
                // Ensure that we actually have an initial surface
                if (frozen_states.empty()) {
                    // TODO we could also just initialize all distances with infinity and just return in this case
                    throw std::runtime_error("There is no surface defined in the input. Can not compute signed distance field");
                }
                // Now do initialization step of algorithm
                for (auto& index : frozen_states) {
                    auto neighbor_gen = fmm_data.grid.getNeighborIndexGenerator(index, 1, 1, 1);
                    while (neighbor_gen.hasNext()) {
                        auto neighbor = neighbor_gen.next();
                        updateDistance(neighbor, fmm_data);
                    }
                }
                // We are ready to perform the main loop
                while (not fmm_data.considered_voxels.empty()) {
                    // get considered voxel with minimal distance
                    auto current_elem = fmm_data.considered_voxels.top();
                    fmm_data.considered_voxels.pop();
                    // freeze it
                    fmm_data.grid(current_elem.index).state = VoxelState::Frozen;
                    // run  over its neighbors and update distances
                    auto neighbor_gen = fmm_data.grid.getNeighborIndexGenerator(current_elem.index, 1, 1, 1);
                    while(neighbor_gen.hasNext()) {
                        auto neighbor = neighbor_gen.next();
                        if (fmm_data.grid(neighbor).state != VoxelState::Frozen) {
                            updateDistance(neighbor, fmm_data);
                        }
                    }
                }
                // we are done, copy signed distances into result grid
                fmm_data.copySignedDistances(result_grid, scale);
            }
        }
    }
}

#endif