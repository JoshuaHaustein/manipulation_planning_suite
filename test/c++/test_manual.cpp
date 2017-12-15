//
// Created by joshua on 8/11/17.
//

#include <iostream>
#include <sim_env/Grid.h>
#include <mps/sdf/FMM.h>

void test_grid(const sim_env::grid::UnsignedIndex& idx, const sim_env::grid::Grid3D<int>& input_grid) {
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
void printGrid(const sim_env::grid::Grid3D<T>& grid) {
    for (size_t z = 0; z < grid.getZSize(); ++z) {
        for (size_t y = 0; y < grid.getYSize(); ++y) {
            for (size_t x = 0; x < grid.getXSize(); ++x) {
                std::cout << grid.at(x, y, z) << ", ";
            }
            std::cout << "\n";
        }
    }
    std::cout << std::endl;
}

void testSDF() {
    Eigen::Vector3f min_pos(-1.2, -1.2, 0.0);
    Eigen::Vector3f max_pos(1.2, 1.2, 0.0);
    sim_env::grid::VoxelGrid<float, int> input_grid(min_pos, max_pos, 0.2, 1);
    sim_env::grid::VoxelGrid<float, float> output_grid(min_pos, max_pos, 0.2);
    // input_grid(0, 0, 0) = -1;
    input_grid(4, 4, 0) = -1;
    input_grid(5, 4, 0) = -1;
    input_grid(6, 4, 0) = -1;
    input_grid(4, 5, 0) = -1;
    input_grid(5, 5, 0) = -1;
    input_grid(6, 5, 0) = -1;
    input_grid(4, 6, 0) = -1;
    input_grid(5, 6, 0) = -1;
    input_grid(6, 6, 0) = -1;
    std::cout << "Input Grid:" << std::endl;
    printGrid(input_grid);
    // test_grid(idx, input_grid);
    // test_grid(idx2, input_grid);
    // test_grid(idx3, input_grid);
    mps::sdf::fmm::computeSDF<float>(input_grid, output_grid, input_grid.getCellSize());
    std::cout << "Output Grid:" << std::endl;
    printGrid(output_grid);
}

int main(int argc, char **argv) {
    testSDF();
}
