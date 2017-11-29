//
// Created by joshua on 8/11/17.
//

#include <iostream>
#include <mps/sdf/Grid.h>
#include <mps/sdf/FMM.h>

void test_grid(const mps::sdf::grid::UnsignedIndex& idx, const mps::sdf::grid::VoxelGrid<int>& input_grid) {
    {
        std::cout << "Block iterator with width 1 in each dimension around " << idx << std::endl;
        auto neighbor_gen = input_grid.getNeighborIndexGenerator(idx, 1, 1, 1);
        while (neighbor_gen.hasNext()) {
            std::cout << neighbor_gen.next() << "\n";
        }
    }
    {
        std::cout << "Blind block iterator with width 1 in each dimension around " << idx << std::endl;
        auto neighbor_gen = input_grid.getBlindNeighborIndexGenerator(idx, 1, 1, 1);
        while (neighbor_gen.hasNext()) {
            std::cout << neighbor_gen.next() << "\n";
        }
    }
    {
        std::cout << "Block iterator with width 1 in x dimension around " << idx << std::endl;
        auto neighbor_gen = input_grid.getNeighborIndexGenerator(idx, 1, 0, 0);
        while (neighbor_gen.hasNext()) {
            std::cout << neighbor_gen.next() << "\n";
        }
    }
    {
        std::cout << "Block iterator with width 2 in y dimension around " << idx << std::endl;
        auto neighbor_gen = input_grid.getNeighborIndexGenerator(idx, 0, 2, 0);
        while (neighbor_gen.hasNext()) {
            std::cout << neighbor_gen.next() << "\n";
        }
    }
    {
        std::cout << "Blind block iterator with width 2 in y dimension around " << idx << std::endl;
        auto neighbor_gen = input_grid.getBlindNeighborIndexGenerator(idx, 0, 2, 0);
        while (neighbor_gen.hasNext()) {
            std::cout << neighbor_gen.next() << "\n";
        }
    }
    {
        std::cout << "Block iterator with width 1 in z dimension around " << idx << std::endl;
        auto neighbor_gen = input_grid.getNeighborIndexGenerator(idx, 0, 0, 2);
        while (neighbor_gen.hasNext()) {
            std::cout << neighbor_gen.next() << "\n";
        }
    }
}

template <typename T>
void printGrid(const mps::sdf::grid::VoxelGrid<T>& grid) {
    for (size_t z = 0; z < grid.z_size; ++z) {
        for (size_t y = 0; y < grid.y_size; ++y) {
            for (size_t x = 0; x < grid.x_size; ++x) {
                std::cout << grid.at(x, y, z) << ", ";
            }
            std::cout << "\n";
        }
    }
    std::cout << std::endl;
}

int main(int argc, char **argv) {
    // std::cout << "Testing voxel grid" << std::endl;
    // mps::sdf::VoxelGrid<float> voxel_grid(10, 10, 1, 1.0f);
    // for (auto& value : voxel_grid) {
    //     std::cout << value;
    // }
    mps::sdf::grid::VoxelGrid<int> input_grid(8, 8, 1, 0);
    mps::sdf::grid::VoxelGrid<float> output_grid(8, 8, 1);
    input_grid(4, 4, 0) = 1;
    input_grid(5, 4, 0) = 1;
    input_grid(6, 4, 0) = 1;
    input_grid(4, 5, 0) = 1;
    input_grid(6, 5, 0) = 1;
    input_grid(4, 6, 0) = 1;
    input_grid(5, 6, 0) = 1;
    input_grid(6, 6, 0) = 1;
    std::cout << "Input Grid:" << std::endl;
    printGrid(input_grid);
    // test_grid(idx, input_grid);
    // test_grid(idx2, input_grid);
    // test_grid(idx3, input_grid);
    mps::sdf::fmm::computeSDF<float>(input_grid, output_grid, 1.0f);
    std::cout << "Output Grid:" << std::endl;
    printGrid(output_grid);
}
