#ifndef MANIPULATION_PLANNING_SUITE_GRID_H
#define MANIPULATION_PLANNING_SUITE_GRID_H

#include <vector>
#include <stdexcept>
#include <ostream>
#include <algorithm>

namespace mps {
    namespace sdf {
        namespace grid {

            template<typename IntegerType>
            struct Index {
                IntegerType ix;
                IntegerType iy;
                IntegerType iz;
                Index(IntegerType x, IntegerType y, IntegerType z) :
                    ix(x), iy(y), iz(z) {}
                Index():ix(0), iy(0), iz(0) {}
                Index(const Index& other) = default;
                virtual ~Index() = default;
                Index& operator=(const Index& other) = default;

                void set(IntegerType x, IntegerType y, IntegerType z) {
                    ix = x;
                    iy = y;
                    iz = z;
                }

                bool operator==(const Index& other) {
                    return other.ix == ix and other.iy == iy and other.iz == iz;
                }

                bool operator!=(const Index& other) {
                    return not operator==(other);
                }

                Index& operator+=(const Index& other) {
                    ix += other.ix;
                    iy += other.iy;
                    iz += other.iz;
                    return *this;
                }

                Index& operator-=(const Index& other) {
                    ix -= other.ix;
                    iy -= other.iy;
                    iz -= other.iz;
                    return *this;
                }

                Index& operator*=(const IntegerType& val) {
                    ix *= val;
                    iy *= val;
                    iz *= val;
                    return *this;
                }

                Index operator+(const Index& other) const {
                    Index new_index(*this);
                    new_index += other;
                    return new_index;
                }

                Index operator-(const Index& other) const {
                    Index new_index(*this);
                    new_index -= other;
                    return new_index;
                }

                Index operator*(const IntegerType& val) const {
                    Index new_index(*this);
                    new_index *= val;
                    return new_index;
                }

                Index<size_t> toUnsignedIndex() const {
                    return Index<size_t>(std::max(IntegerType(0), ix),
                                         std::max(IntegerType(0), iy),
                                         std::max(IntegerType(0), iz));
                }
            };

            typedef Index<size_t> UnsignedIndex;
            typedef Index<long> SignedIndex;

            template<typename IntegerType>
            std::ostream& operator<<(std::ostream& os, const Index<IntegerType>& idx) {
                os << "(" << idx.ix << ", " << idx.iy << ", " << idx.iz << ")";
                return os;
            }


            /**
             *  A IndexGenerator allows you to space efficiently iterate over all possible indices
             * of a grid. Create it with the dimensions of the grid, then by calling next() repeatedly, you
             * can generate all grid indices successively. hasNext() returns whether the generator sequence
             * has reached its end.
             */
            template<typename IntegerType>
            struct IndexGenerator {
                private:
                    const size_t _x_max;
                    const size_t _y_max;
                    const size_t _z_max;
                    Index<IntegerType> _current_idx;
                public:
                    IndexGenerator(const size_t& max_x, const size_t& max_y, const size_t& max_z) :
                        _x_max(max_x), _y_max(max_y), _z_max(max_z) {}

                    Index<IntegerType> next() {
                        Index<IntegerType> old_index(_current_idx);
                        _current_idx.ix += 1;
                        size_t overflow = _current_idx.ix / _x_max;
                        _current_idx.ix = _current_idx.ix % _x_max;
                        _current_idx.iy += overflow;
                        overflow = _current_idx.iy / _y_max;
                        _current_idx.iy = _current_idx.iy % _y_max;
                        _current_idx.iz += overflow;
                        return old_index;
                    }

                    bool hasNext() {
                        return _current_idx.iz < _z_max;
                    }
            };

            typedef IndexGenerator<size_t> UnsignedIndexGenerator;
            typedef IndexGenerator<long> SignedIndexGenerator;

            /**
             * Similar to an IndexGenerator, a BoxIndexGenerator can be used to generate a sequence
             * of grid indices space efficiently. In contrast to the normal IndexGenerator, this generator
             * produces indices in a box centered around a given index.
             */
            template<typename IntegerType>
            struct BoxIndexGenerator {
                private:
                    const size_t _x_max;
                    const size_t _y_max;
                    const size_t _z_max;
                    SignedIndex _base_idx;
                    IndexGenerator<long> _relative_idx_gen;
                    SignedIndex _next_idx;
                    bool _next_idx_valid;
                public:
                    BoxIndexGenerator(const size_t& max_x, const size_t& max_y, const size_t& max_z,
                                   const Index<IntegerType>& center_idx,
                                   const size_t& dx, const size_t& dy, const size_t& dz) :
                        _x_max(max_x), _y_max(max_y), _z_max(max_z),
                        _base_idx((long)center_idx.ix - dx, (long)center_idx.iy - dy, (long)center_idx.iz - dz),
                        _relative_idx_gen(2 * dx + 1, 2 * dy + 1, 2 * dz + 1)
                    {
                        next();
                    }

                    Index<IntegerType> next() {
                        // _next_idx.ix/iy/iz are >= 0
                        Index<IntegerType> last_index(_next_idx.ix, _next_idx.iy, _next_idx.iz);
                        _next_idx_valid = false;
                        while (_relative_idx_gen.hasNext())
                        {
                            _next_idx = _relative_idx_gen.next() + _base_idx;
                            if (isValid(_next_idx)) {
                                _next_idx_valid = true;
                                break;
                            }
                        }
                        return last_index;
                    }

                    bool hasNext() {
                        return _next_idx_valid;
                    }

                private:
                    inline bool isValid(const SignedIndex& idx) {
                        return idx.ix >= 0 and idx.iy >= 0 and idx.iz >= 0
                            and idx.ix < _x_max and idx.iy < _y_max and idx.iz < _z_max;
                    }
            };
            typedef BoxIndexGenerator<size_t> UnsignedBoxIndexGenerator;
            typedef BoxIndexGenerator<long> SignedBoxIndexGenerator;

            /**
             * A BlindBoxIndexGenerator works in the same way as a BoxIndexGenerator, except that this
             * generator also creates indices out of range of the grid. Use with care!
             */
            struct BlindBoxIndexGenerator {
                private:
                    const size_t _dx;
                    const size_t _dy;
                    const size_t _dz;
                    IndexGenerator<long> _relative_idx_gen;
                    const SignedIndex _base_idx;
                public:
                    BlindBoxIndexGenerator(const long& dx, const long& dy, const long& dz,
                                           const long& cx, const long& cy, const long& cz) :
                        _dx(dx + 1), _dy(dy + 1), _dz(dz + 1),
                        _relative_idx_gen(2 * dx + 1, 2 * dy + 1, 2* dz + 1),
                        _base_idx(cx - dx, cy - dy, cz - dz)
                    {
                    }
                    BlindBoxIndexGenerator(const long& dx, const long& dy, const long& dz,
                                           const SignedIndex& center_idx) :
                        BlindBoxIndexGenerator(dx, dy, dz, center_idx.ix, center_idx.iy, center_idx.iz)
                    {
                    }

                    BlindBoxIndexGenerator(const long& dx, const long& dy, const long& dz,
                                           const UnsignedIndex& center_idx) :
                        BlindBoxIndexGenerator(dx, dy, dz, (long)center_idx.ix, (long)center_idx.iy, (long)center_idx.iz)
                    {
                    }

                    SignedIndex next() {
                        return _relative_idx_gen.next() + _base_idx;
                    }

                    bool hasNext() {
                        return _relative_idx_gen.hasNext();
                    }
            };

            /**
             * A simple implementation of a voxel grid that stores values of type ValueType
             */
            template <typename ValueType>
            class VoxelGrid {
                public:
                    const size_t x_size;
                    const size_t y_size;
                    const size_t z_size;

                private:
                    std::vector<ValueType> _values;
                    const size_t _xy_stride;
                    inline size_t getFlatIndex(const size_t& x, const size_t& y, const size_t& z) const {
                        return x + y * y_size + z * _xy_stride;
                    }
                public:
                    VoxelGrid(size_t max_x, size_t max_y, size_t max_z):
                        x_size(max_x), y_size(max_y), z_size(max_z), _xy_stride(x_size * y_size)
                    {
                        _values.resize(x_size * y_size * z_size);
                    }
                    VoxelGrid(size_t max_x, size_t max_y, size_t max_z, ValueType default_value) :
                        x_size(max_x), y_size(max_y), z_size(max_z), _xy_stride(x_size * y_size)
                    {
                        _values.resize(x_size * y_size * z_size, default_value);
                    }

                    VoxelGrid(const VoxelGrid<ValueType>& other) = default;
                    ~VoxelGrid() = default;
                    VoxelGrid<ValueType>& operator=(const VoxelGrid<ValueType>& other) = default;

                    inline bool inBounds(const size_t& ix, const size_t& iy, const size_t& iz) const {
                        return ix < x_size && iy < y_size && iz < z_size;
                    }

                    inline bool inBounds(const long& ix, const long& iy, const long& iz) const {
                        return ix >= 0 and iy >= 0 and iz >= 0 and ix < x_size and iy < y_size and iz < z_size;
                    }

                    inline bool inBounds(const SignedIndex& idx) const {
                        return inBounds(idx.ix, idx.iy, idx.iz);
                    }

                    inline bool inBounds(const UnsignedIndex& idx) const {
                        return inBounds(idx.ix, idx.iy, idx.iz);
                    }

                    ValueType& operator()(const size_t& ix, const size_t& iy, const size_t& iz) {
                        return _values[getFlatIndex(ix, iy, iz)];
                    }

                    ValueType& operator()(const UnsignedIndex& idx) {
                        return operator()(idx.ix, idx.iy, idx.iz);
                    }

                    const ValueType& operator()(const size_t& ix, const size_t& iy, const size_t& iz) const {
                        return _values[getFlatIndex(ix, iy, iz)];
                    }

                    const ValueType& operator()(const UnsignedIndex& idx) const {
                        return operator()(idx.ix, idx.iy, idx.iz);
                    }

                    ValueType& at(const size_t& ix, const size_t& iy, const size_t& iz) {
                        if (not inBounds(ix, iy, iz)) {
                            throw std::out_of_range("The provided index is out of range for this grid.");
                        }
                        return _values[getFlatIndex(ix, iy, iz)];
                    }

                    ValueType& at(const long& ix, const long& iy, const long& iz) {
                        if (not inBounds(ix, iy, iz)) {
                            throw std::out_of_range("The provided index is out of range for this grid.");
                        }
                        return _values[getFlatIndex(ix, iy, iz)];
                    }

                    ValueType& at(const SignedIndex& idx) {
                        return at(idx.ix, idx.iy, idx.iz);
                    }

                    ValueType& at(const UnsignedIndex& idx) {
                        return at(idx.ix, idx.iy, idx.iz);
                    }

                    const ValueType& at(const size_t& ix, const size_t& iy, const size_t& iz) const {
                        if (not inBounds(ix, iy, iz)) {
                            throw std::out_of_range("The provided index is out of range for this grid.");
                        }
                        return _values[getFlatIndex(ix, iy, iz)];
                    }

                    const ValueType& at(const long& ix, const long& iy, const long& iz) const {
                        if (not inBounds(ix, iy, iz)) {
                            throw std::out_of_range("The provided index is out of range for this grid.");
                        }
                        return _values[getFlatIndex(ix, iy, iz)];
                    }

                    const ValueType& at(const UnsignedIndex& idx) const {
                        return at(idx.ix, idx.iy, idx.iz);
                    }

                    const ValueType& at(const SignedIndex& idx) const {
                        return at(idx.ix, idx.iy, idx.iz);
                    }

                    UnsignedIndexGenerator getIndexGenerator() const {
                        return UnsignedIndexGenerator(x_size, y_size, z_size);
                    }

                    UnsignedBoxIndexGenerator getNeighborIndexGenerator(const UnsignedIndex& idx,
                                                                        const size_t& dx,
                                                                        const size_t& dy,
                                                                        const size_t& dz) const {
                        return UnsignedBoxIndexGenerator(x_size, y_size, z_size, idx, dx, dy, dz);
                    }

                    BlindBoxIndexGenerator getBlindNeighborIndexGenerator(const UnsignedIndex& idx,
                                                                          const size_t& dx,
                                                                          const size_t& dy,
                                                                          const size_t& dz) const {
                        return BlindBoxIndexGenerator(dx, dy, dz, idx);
                    }
                    /**
                     *  Returns all neighbors of the provided index. Boundaries are considered,
                     *  i.e. neighbors will only include valid indices for this grid (empty if idx is invalid)
                     * @param idx - index to get neighbors of
                     * @param neigbors - vector of indices, gets cleared, contains output
                     */
                    // void getNeighbors(const Index& idx, std::vector<Index>& neighbors) const {
                    //     neighbors.clear();
                    //     for (int dz = -1; dz <= 1; ++dz) {
                    //         int nz = idx.iz + dz;
                    //         for (int dy = -1; dy <= 1; ++dy) {
                    //             int ny = idx.iy + dy;
                    //             for (int dx = -1; dx <= 1; ++dx) {
                    //                 int nx = idx.ix + dx;
                    //                 if (inBounds(nx, ny, nz)) {
                    //                     neighbors.emplace_back(Index(nx, ny, nz));
                    //                 }
                    //             }
                    //         }
                    //     }
                    // }

                    typename std::vector<ValueType>::iterator begin() noexcept {
                        return _values.begin();
                    }

                    typename std::vector<ValueType>::const_iterator begin() const noexcept {
                        return _values.begin();
                    }

                    typename std::vector<ValueType>::const_iterator cbegin() const noexcept {
                        return _values.cbegin();
                    }

                    typename std::vector<ValueType>::iterator end() noexcept {
                        return _values.end();
                    }

                    typename std::vector<ValueType>::const_iterator end() const noexcept {
                        return _values.end();
                    }

                    typename std::vector<ValueType>::const_iterator cend() const noexcept {
                        return _values.cend();
                    }

                    typename std::vector<ValueType>::reverse_iterator rbegin() noexcept {
                        return _values.rbegin();
                    }

                    typename std::vector<ValueType>::const_reverse_iterator rbegin() const noexcept {
                        return _values.rbegin();
                    }

                    typename std::vector<ValueType>::const_reverse_iterator rcbegin() const noexcept {
                        return _values.rcbegin();
                    }

                    typename std::vector<ValueType>::reverse_iterator rend() noexcept {
                        return _values.rend();
                    }

                    typename std::vector<ValueType>::const_reverse_iterator rend() const noexcept {
                        return _values.rend();
                    }

                    typename std::vector<ValueType>::const_reverse_iterator rcend() const noexcept {
                        return _values.rcend();
                    }
            };
        }
    }
}


#endif //MANIPULATION_PLANNING_SUITE_GRID_H