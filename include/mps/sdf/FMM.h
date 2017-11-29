
#ifndef MANIPULATION_PLANNING_SUITE_FMM_H
#define MANIPULATION_PLANNING_SUITE_FMM_H

#include <vector>
#include <stdexcept>
#include <iostream>
#include <math.h>

// #include <boost/heap/fibonacci_heap.cpp>
#include <boost/heap/binomial_heap.hpp>

namespace mps {
    namespace sdf {
        namespace fmm {
            // State of voxels in the fmm algorithm
            enum VoxelState {Frozen, Considered, Far};
            template<typename ScalarType>
            struct PriorityQueueElement {
                ScalarType value;
                grid::UnsignedIndex index;
                PriorityQueueElement(const ScalarType& val, const grid::UnsignedIndex& idx) : value(val), index(idx) {}
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
            /**
             * Black board data structure for fmm algorithm.
             */
            template<typename ScalarType>
            struct FMMData {
                typedef boost::heap::binomial_heap<PriorityQueueElement<ScalarType> , boost::heap::stable<false>, boost::heap::compare<PriorityQueueElementComparator<ScalarType> > > PriorityQueue;
                PriorityQueue considered_voxels;
                const grid::VoxelGrid<int>& input_grid;
                grid::VoxelGrid<ScalarType>& result_grid;
                grid::VoxelGrid<VoxelState> voxel_states;
                grid::VoxelGrid<typename PriorityQueue::handle_type> pq_handles;
                FMMData(const grid::VoxelGrid<int>& grid_in, grid::VoxelGrid<ScalarType>& grid_out) :
                    input_grid(grid_in),
                    result_grid(grid_out),
                    voxel_states(grid_out.x_size, grid_out.y_size, grid_out.z_size),
                    pq_handles(grid_out.x_size, grid_out.y_size, grid_out.z_size) {}
                FMMData(const FMMData& other) = default;
                ~FMMData() = default;
            };

            // Returns true if both grids have same dimensions, else false
            template<typename T_1, typename T_2>
            bool compareGridSize(const grid::VoxelGrid<T_1>& grid_1, const grid::VoxelGrid<T_2>& grid_2) {
                if (grid_1.x_size != grid_2.x_size ||
                    grid_1.y_size != grid_2.y_size ||
                    grid_1.z_size != grid_2.z_size)
                {
                    return false;
                }
                return true;
            }

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
            void computeCoefficients(const grid::UnsignedIndex& idx,
                                     std::array<ScalarType, 3>& coeffs,
                                     FMMData<ScalarType>& fmm_data) {
                std::array<grid::SignedIndex, 6> directions = {{grid::SignedIndex(-1, 0, 0),
                                                               grid::SignedIndex(1, 0, 0),
                                                               grid::SignedIndex(0, -1, 0),
                                                               grid::SignedIndex(0, 1, 0),
                                                               grid::SignedIndex(0, 0, -1),
                                                               grid::SignedIndex(0, 0, 1)}};
                grid::SignedIndex center_idx(idx.ix, idx.iy, idx.iz);
                for (unsigned int r = 0; r < 3; ++r) { // for each axis x, y, z
                    // we need to figure whether we take forward or backward gradient
                    // as well as whether to use first order or second order derivative
                    ScalarType val_1(std::numeric_limits<ScalarType>::max());
                    ScalarType val_2(std::numeric_limits<ScalarType>::max());
                    for (unsigned int c = 0; c < 2; ++c) { // for forward and backward direction
                        auto d1_idx = center_idx + directions[r * 2 + c]; // idx - 1
                        if (fmm_data.voxel_states.inBounds(d1_idx) and
                            fmm_data.voxel_states(d1_idx.toUnsignedIndex()) == VoxelState::Frozen) {
                            ScalarType tval_1 = fmm_data.result_grid(d1_idx.toUnsignedIndex()); // idx - 1 is valid, so store value
                            if (tval_1 < val_1) { // if it is smaller than for the other direction
                                val_1 = tval_1;
                                // let's see whether we can get a second order derivative
                                auto d2_idx = center_idx + directions[r * 2 + c] * 2;
                                if (fmm_data.voxel_states.inBounds(d2_idx) and
                                    fmm_data.voxel_states(d2_idx.toUnsignedIndex()) == VoxelState::Frozen) {
                                    ScalarType tval_2 = fmm_data.result_grid(d2_idx.toUnsignedIndex());
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

            // template<typename ScalarType>
            // inline void add_coeffs(const grid::UnsignedIndex& idx, const std::array<size_t, 3>& dir,
            //                        std::array<ScalarType, 3>& coeffs, FMMData<ScalarType>& fmm_data) {
            //     auto nn_gen = fmm_data.result_grid.getBlindNeighborIndexGenerator(idx, dir[0], dir[1], dir[2]);
            //     std::array<std::pair<bool, ScalarType>, 6> entry_info;
            //     size_t array_idx = 0;
            //     while (nn_gen.hasNext()) {
            //         auto grid_idx = nn_gen.next();
            //         if (fmm_data.voxel_states.inBounds(grid_idx)) {
            //             entry_info[array_idx].get<bool>() = fmm_data.voxel_states(grid_idx) == VoxelState::Frozen;
            //             entry_info[array_idx].get<ScalarType>() = fmm_data.result_grid(grid_idx);
            //         } else {
            //             entry_info[array_idx].get<bool>() = false;
            //             entry_info[array_idx].get<ScalarType>() = 0.0;
            //         }
            //     }
            //     if (entry_info[0].get<bool>()) {
            //         entry_info[0].get<bool>() = entry_info[1].get<bool>() and
            //                                     entry_info[0].get<ScalarType>() < entry_info[1].get<ScalarType>();
            //     }
            //     if (entry_info[5].get<bool>()) {
            //         entry_info[5].get<bool>() = entry_info[4].get<bool>() and
            //                                     entry_info[5].get<ScalarType>() < entry_info[4].get<ScalarType>();
            //     }
            // }
            /**
             * Computes the minimal distance to the intial condition for the voxel with index idx.
             * For the distance computation only the frozen neighbors are taken into account.
             * Thus, if idx has no frozen neighbors, the distance is not updated. In any case
             * the best guess of the minimal distance is returned.
             */
            template<typename ScalarType>
            ScalarType computeDistance(const grid::UnsignedIndex& idx, FMMData<ScalarType>& fmm_data) {
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

            template<typename ScalarType>
            void updateDistance(const grid::UnsignedIndex& idx, FMMData<ScalarType>& fmm_data) {
                if (fmm_data.voxel_states(idx) == VoxelState::Frozen) {
                    return;
                }
                ScalarType dist = computeDistance<ScalarType>(idx, fmm_data);
                if (fmm_data.voxel_states(idx) == VoxelState::Considered) {
                    fmm_data.considered_voxels.decrease(fmm_data.pq_handles(idx), PriorityQueueElement<ScalarType>(dist, idx));
                    fmm_data.result_grid(idx) = dist;
                } else if (dist != std::numeric_limits<ScalarType>::max()) {
                    assert(fmm_data.voxel_states(idx) == VoxelState::Far);
                    fmm_data.voxel_states(idx) = VoxelState::Considered;
                    fmm_data.pq_handles(idx) = fmm_data.considered_voxels.emplace(PriorityQueueElement<ScalarType>(dist, idx));
                    fmm_data.result_grid(idx) = dist;
                }
            }

            template<typename ScalarType>
            void computeSDF(const grid::VoxelGrid<int>& binary_grid,
                            grid::VoxelGrid<ScalarType>& result_grid,
                            ScalarType scale=ScalarType(1))
            {
                if (not compareGridSize(binary_grid, result_grid)) {
                    throw std::logic_error("Input and output grid do not have the same dimension.");
                }
                FMMData<ScalarType> fmm_data(binary_grid, result_grid);
                // Initialize data structures
                auto idx_generator = binary_grid.getIndexGenerator();
                std::vector<grid::UnsignedIndex> frozen_states;
                while (idx_generator.hasNext()) {
                    auto index = idx_generator.next();
                    if (binary_grid(index)) {  // index is a boundary cell
                        fmm_data.voxel_states(index) = VoxelState::Frozen;
                        fmm_data.result_grid(index) = ScalarType(0);
                        frozen_states.push_back(index);
                    } else {
                        fmm_data.voxel_states(index) = VoxelState::Far;
                    }
                }
                // Ensure that we actually have an initial surface
                if (frozen_states.empty()) {
                    throw std::runtime_error("There is no surface defined in the input. Can not compute signed distance field");
                }
                // Now do initialization step of algorithm
                for (auto& index : frozen_states) {
                    auto neighbor_gen = result_grid.getNeighborIndexGenerator(index, 1, 1, 1);
                    while (neighbor_gen.hasNext()) {
                        auto neighbor = neighbor_gen.next();
                        updateDistance(neighbor, fmm_data);
                    }
                }
                // We are ready to perform the main loop
                while (not fmm_data.considered_voxels.empty()) {
                    auto current_elem = fmm_data.considered_voxels.top();
                    fmm_data.considered_voxels.pop();
                    fmm_data.voxel_states(current_elem.index) = VoxelState::Frozen;
                    auto neighbor_gen = result_grid.getNeighborIndexGenerator(current_elem.index, 1, 1, 1);
                    while(neighbor_gen.hasNext()) {
                        auto neighbor = neighbor_gen.next();
                        if (fmm_data.voxel_states(neighbor) != VoxelState::Frozen) {
                            updateDistance(neighbor, fmm_data);
                        }
                    }
                }
            }
        }
    }
}

#endif