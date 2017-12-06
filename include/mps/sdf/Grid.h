#ifndef MANIPULATION_PLANNING_SUITE_GRID_H
#define MANIPULATION_PLANNING_SUITE_GRID_H

#include <vector>
#include <stdexcept>
#include <ostream>
#include <algorithm>
#include <Eigen/Geometry>

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

                Index& operator/=(const IntegerType& val) {
                    ix /= val;
                    iy /= val;
                    iz /= val;
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

                Index operator/(const IntegerType& val) const {
                    Index new_index(*this);
                    new_index /= val;
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


            /*
             * A simple implementation of a 3D grid that stores values of type ValueType. ValueType can be
             * any type that can be stored in a std::vector.
             */
            template <typename ValueType>
            class Grid3D {
                public:
                    template<typename ValueType1>
                    friend std::ostream& operator<<(std::ostream& os, Grid3D<ValueType1> const& grid);
                    template<typename ValueType1>
                    friend std::istream& operator>>(std::istream& is, Grid3D<ValueType1>& grid);
                private:
                    std::vector<ValueType> _values;
                    size_t _x_size;
                    size_t _y_size;
                    size_t _z_size;
                    size_t _xy_stride;
                    inline size_t getFlatIndex(const size_t& x, const size_t& y, const size_t& z) const {
                        return x + y * _y_size + z * _xy_stride;
                    }
                protected:
                    /*
                     * Reset this grid to a new size.
                     * NOTE: All generated indices may become invalid and all stored data
                     * has to be considered lost.
                     */
                    void reset(size_t new_x, size_t new_y, size_t new_z, ValueType default_value=ValueType()) {
                        _x_size = new_x;
                        _y_size = new_y;
                        _z_size = new_z;
                        _xy_stride = _x_size * _y_size;
                        _values.resize(_x_size * _y_size * _z_size, default_value);
                    }
                public:
                    Grid3D(size_t max_x, size_t max_y, size_t max_z):
                        _x_size(max_x), _y_size(max_y), _z_size(max_z), _xy_stride(_x_size * _y_size)
                    {
                        _values.resize(_x_size * _y_size * _z_size);
                    }
                    Grid3D(size_t max_x, size_t max_y, size_t max_z, ValueType default_value) :
                        _x_size(max_x), _y_size(max_y), _z_size(max_z), _xy_stride(_x_size * _y_size)
                    {
                        _values.resize(_x_size * _y_size * _z_size, default_value);
                    }

                    Grid3D(const Grid3D<ValueType>& other) = default;
                    ~Grid3D() = default;
                    Grid3D<ValueType>& operator=(const Grid3D<ValueType>& other) = default;

                    inline size_t getXSize() const {
                        return _x_size;
                    }

                    inline size_t getYSize() const {
                        return _y_size;
                    }

                    inline size_t getZSize() const {
                        return _z_size;
                    }

                    inline bool inBounds(const size_t& ix, const size_t& iy, const size_t& iz) const {
                        return ix < _x_size && iy < _y_size && iz < _z_size;
                    }

                    inline bool inBounds(const long& ix, const long& iy, const long& iz) const {
                        return ix >= 0 and iy >= 0 and iz >= 0 and ix < _x_size and iy < _y_size and iz < _z_size;
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
                        return UnsignedIndexGenerator(_x_size, _y_size, _z_size);
                    }

                    UnsignedBoxIndexGenerator getNeighborIndexGenerator(const UnsignedIndex& idx,
                                                                        const size_t& dx,
                                                                        const size_t& dy,
                                                                        const size_t& dz) const {
                        return UnsignedBoxIndexGenerator(_x_size, _y_size, _z_size, idx, dx, dy, dz);
                    }

                    BlindBoxIndexGenerator getBlindNeighborIndexGenerator(const UnsignedIndex& idx,
                                                                          const size_t& dx,
                                                                          const size_t& dy,
                                                                          const size_t& dz) const {
                        return BlindBoxIndexGenerator(dx, dy, dz, idx);
                    }

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

            // Operator for convenient output of Grid3Ds; needs ValueType to be streamable
            template <typename ValueType>
            inline std::ostream& operator<<(std::ostream& os, mps::sdf::grid::Grid3D<ValueType> const& grid) {
                os << grid.getXSize() << grid.getYSize() << grid.getZSize();
                for(const auto& value : grid) {
                    os << value;
                }
                return os;
            }

            // Operator for convenient input of Grid3Ds; needs ValueType to be streamable
            template <typename ValueType>
            inline std::istream& operator>>(std::istream& is, mps::sdf::grid::Grid3D<ValueType>& grid) {
                size_t x_size;
                is >> x_size;
                size_t y_size;
                is >> y_size;
                size_t z_size;
                is >> z_size;
                grid.reset(x_size, y_size, z_size);
                for (auto& value : grid) {
                    is >> value;
                }
                return is;
            }

            /**
             * A VoxelGrid extends the functionality of Grid3D by spatial relations. Each
             * cell of the grid represents a volume in R^3. The size of the volume is determined
             * by the size of the voxel (which is identical in each dimension). Furthermore,
             * a voxel grid provides a transformation from some world frame to its local frame.
             */
            template<typename ScalarType, typename ValueType>
            class VoxelGrid : public grid::Grid3D<ValueType> {
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                public:
                    template<typename ScalarType1, typename ValueType1>
                    friend std::ostream& operator<<(std::ostream& os, VoxelGrid<ScalarType1, ValueType1> const& grid);
                    template<typename ScalarType1, typename ValueType1>
                    friend std::istream& operator>>(std::istream& is, VoxelGrid<ScalarType1, ValueType1>& grid);
                    typedef Eigen::Matrix<ScalarType, 3, 1> Vector3s;
                private:
                    ScalarType _cell_size;
                    Eigen::Transform<ScalarType, 3, Eigen::Affine> _transform;
                    Eigen::Transform<ScalarType, 3, Eigen::Affine> _inv_transform;
                    Vector3s _min_point;
                    Vector3s _max_point;
                    std::array<size_t, 3> _num_cells;
                protected:
                    void resetVoxelGrid(const Vector3s& min_point, const Vector3s& max_point,
                                const ScalarType& cell_size, ValueType default_value=ValueType())
                    {
                        _cell_size = cell_size;
                        _min_point = min_point;
                        _max_point = max_point;
                        auto dimensions = _max_point - _min_point;
                        _num_cells[0] = (size_t)std::max(std::ceil(dimensions[0] / cell_size), ScalarType(1.0));
                        _num_cells[1] = (size_t)std::max(std::ceil(dimensions[1] / cell_size), ScalarType(1.0));
                        _num_cells[2] = (size_t)std::max(std::ceil(dimensions[2] / cell_size), ScalarType(1.0));
                        _transform.setIdentity();
                        _inv_transform.setIdentity();
                        this->reset(_num_cells[0], _num_cells[1], _num_cells[2], default_value);
                    }

                public:
                    VoxelGrid(const Vector3s& min_point, const Vector3s& max_point,
                              const ScalarType& cell_size, ValueType default_value=ValueType()) :
                        Grid3D<ValueType>(1, 1, 1)
                    {
                        resetVoxelGrid(min_point, max_point, cell_size, default_value);
                    }

                    VoxelGrid(const VoxelGrid& other) = default;
                    ~VoxelGrid() = default;
                    VoxelGrid& operator=(const VoxelGrid& other) = default;

                    /*
                    * Returns the index of the voxel in which the specified position in world frame lies.
                    * Note that the returned index may be out of bounds, if the position is out of bounds.
                    * You can check this by calling inBounds(idx). Alternatively, use getValidCellIdx(..)
                    */
                    SignedIndex getCellIdx(const Vector3s& position) const {
                        auto local_pos = _inv_transform * position;
                        local_pos -= _min_point;
                        SignedIndex idx(local_pos[0] / _cell_size, local_pos[1] / _cell_size, local_pos[2] / _cell_size);
                        return idx;
                    }

                    /*
                    * Returns the index of the voxel in which the specified position in world frame lies.
                    * @param position - the query position
                    * @param is_valid - flag that is set to true, if the returned index is valid, else false
                    * @return - an index that is either value or undefined depending on is_valid flag
                    */
                    UnsignedIndex getValidCellIdx(const Vector3s& position, bool& is_valid) const {
                        SignedIndex idx = getCellIdx(position);
                        is_valid = this->inBounds(idx);
                        if (is_valid) {
                            return idx.toUnsignedIndex();
                        }
                        return UnsignedIndex();
                    }

                    /*
                    * Maps the given position to the local frame of this voxel grid and computes the
                    * voxel index for that position.
                    * @param in_position - query position in world frame
                    * @param out_pos - in_position transformed to local frame
                    * @param idx - index of the voxel containing out_pos, if out_pos is within bounds
                    * @return whether out_pos is within bounds
                    */
                    bool mapToGrid(const Vector3s& in_position, Vector3s& out_pos, UnsignedIndex& idx) const {
                        auto local_pos = _inv_transform * in_position;
                        out_pos = local_pos;
                        local_pos -= _min_point;
                        SignedIndex sidx(local_pos[0] / _cell_size, local_pos[1] / _cell_size, local_pos[2] / _cell_size);
                        bool is_valid = this->inBounds(sidx);
                        if (is_valid) {
                            idx = sidx.toUnsignedIndex();
                            return true;
                        }
                        return false;
                    }

                    /*
                    * Returns the position in R^3 of the center or min corner of the voxel with index idx
                    * @param idx - a valid cell index
                    * @param position - output position of either the center or min corner position of the voxel
                    * @param b_center - if true, position is the position of the center, else of min corner
                    */
                    void getCellPosition(const UnsignedIndex& idx, Vector3s& position, bool b_center) const {
                        position[0] = idx.ix * _cell_size;
                        position[1] = idx.iy * _cell_size;
                        position[2] = idx.iz * _cell_size;
                        position += _min_point;
                        if (b_center) position += Vector3s(_cell_size / 2.0, _cell_size / 2.0, _cell_size / 2.0);
                        position = _transform * position;
                    }

                    ScalarType getCellSize() const {
                        return _cell_size;
                    }

                    /**
                     * Returns the local axis aligned bounding box of this grid.
                     * This is essentially the bounding box passed to the constructor.
                     */
                    void getBoundingBox(Vector3s& min_point, Vector3s& max_point) const {
                        min_point = _min_point;
                        max_point = _max_point;
                    }

                    /**
                     * Sets the transformation for this grid. The given transformation is expected
                     * to only consist of a rotation and translation.
                     */
                    void setTransform(const Eigen::Transform<ScalarType, 3, Eigen::Affine>& tf) {
                        _transform = tf;
                        _inv_transform = _transform.inverse();
                        // TODO there is an easy way to compute this inverse, but Eigen makes this unnecessary difficult.
                        // _inv_transform.matrix().block(0, 0, 3, 3) = tf.matrix().transpose().block(0, 0, 3, 3);
                        // Vector3s translation(tf.translation.x(), tf.translation.y(), tf.translation.z());
                        // auto translation = _inv_transform.matrix().block(0, 0, 3, 3) * tf.translation()
                        // _inv_transform.matrix().block(0, 3, 3, 1) = translation;
                    }

                    Eigen::Transform<ScalarType, 3, Eigen::Affine> getTransform() const {
                        return _transform;
                    }

                    Eigen::Transform<ScalarType, 3, Eigen::Affine> getInvTransform() const {
                        return _inv_transform;
                    }
            };

            // Operator for convenient output of VoxelGrids; needs ScalarType and ValueType to be streamable
            template <typename ScalarType, typename ValueType>
            inline std::ostream& operator<<(std::ostream& os, mps::sdf::grid::VoxelGrid<ScalarType, ValueType> const& grid) {
                Eigen::Matrix<ScalarType, 3, 1> min_pos;
                Eigen::Matrix<ScalarType, 3, 1> max_pos;
                grid.getBoundingBox(min_pos, max_pos);
                os << min_pos[0] << min_pos[1] << min_pos[2];
                os << max_pos[0] << max_pos[1] << max_pos[2];
                os << grid.getCellSize();
                for(const auto& value : grid) {
                    os << value;
                }
                return os;
            }

            // Operator for convenient input of VoxelGrids; needs ScalarType and ValueType to be streamable
            template <typename ScalarType, typename ValueType>
            inline std::istream& operator>>(std::istream& is, mps::sdf::grid::VoxelGrid<ScalarType, ValueType>& grid) {
                Eigen::Matrix<ScalarType, 3, 1> min_pos;
                Eigen::Matrix<ScalarType, 3, 1> max_pos;
                is >> min_pos[0] >> min_pos[1] >> min_pos[2];
                is >> max_pos[0] >> max_pos[1] >> max_pos[2];
                ScalarType cell_size;
                is >> cell_size;
                grid.resetVoxelGrid(min_pos, max_pos, cell_size);
                for (auto& value : grid) {
                    is >> value;
                }
                return is;
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_GRID_H