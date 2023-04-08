#include <surface_net.hpp>

constexpr int _empty = -1;
constexpr int _filled = -2;
constexpr std::array<uint16_t, 6> vOrder = {1,3,0,3,2,0};

// Compute surface vertex positions from data
void meshNet(const Vector<float, chunkCells>& data, std::vector<Vector3f>& vertices, std::vector<uint32_t>& tris) {
    // Contains vertex indices for each cell
    Vector<int, chunkCells> idxBuf;
    idxBuf.fill(_empty);

    // Sweep through volume placing vertices
    for (uint16_t z = 0; z < chunkSize - 1u; z++)
    for (uint16_t x = 0; x < chunkSize - 1u; x++)
    for (uint16_t y = 0; y < chunkSize - 1u; y++) {
        uint16_t occupancy = 0u;
        for (uint16_t i = 0; i < 2; i++)
        for (uint16_t j = 0; j < 2; j++)
        for (uint16_t k = 0; k < 2; k++) {
            if (data[flatIdx(x+i, y+j, z+k)] > 0.0f) {
                occupancy += 1u;
            }
        }
        if (occupancy == 8u) {
            idxBuf[flatIdx(x, y, z)] = _filled;
        } else if (occupancy != 0u) {
            idxBuf[flatIdx(x, y, z)] = vertices.size();
            // Linear interpolate the vertex based on the data
            Vector3f v = {0.0f, 0.0f, 0.0f};
            for (uint16_t i = 0; i < 2; i++)
            for (uint16_t j = 0; j < 2; j++)
            for (uint16_t k = 0; k < 2; k++) {
                if (data[flatIdx(x+i, y+j, z+k)] > 0.0f) {
                    v += Vector3f{(float)(x+i), (float)(y+j), (float)(z+k)};
                }
            }
            v /= (float)occupancy;
            vertices.push_back(v);
        }
    }

    for (int dir : {-1, 1}) {
        uint16_t d0, d1;
        if (dir == 1) {
            d0 = 1;
            d1 = chunkSize;
        } else {
            d0 = chunkSize - 2;
            d1 = -1;
        }
        for (uint16_t axis = 0; axis < 3; axis++)
        for (uint16_t z = d0; z != d1; z += dir)
        for (uint16_t x = d0; x != d1; x += dir)
        for (uint16_t y = d0; y != d1; y += dir) {
            const std::array<uint16_t, 3> c = {x, y, z};

            int fIdx[4];
            bool valid = true;
            for (uint16_t i = 0; i < 4; i++) {
                // Face coordinate
                std::array<uint16_t, 3> fc = c;
                fc[(axis + 0) % 3] += i / 2;
                fc[(axis + 1) % 3] += i % 2;
                // Previous face coordinate, invalidate face if its filled
                std::array<uint16_t, 3> pfc = fc;
                pfc[(axis + 2) % 3] -= dir;
                if (idxBuf[flatIdx(pfc[0], pfc[1], pfc[2])] == _filled) {
                    valid = false;
                    break;
                }
                // Check current face
                int idx = idxBuf[flatIdx(fc[0], fc[1], fc[2])];
                if (idx < 0) {
                    valid = false;
                    break;
                } else {
                    fIdx[i] = idx;
                }
            }

            // Construct triangles
            if (valid) {
                for (size_t i = 0; i < 6; i++) {
                    tris.push_back(fIdx[vOrder[dir == 1 ? i : 5 - i]]);
                }
            }
        }
    }
}

void SurfaceNet::build(const Vector<float, chunkCells>& data) {
    meshNet(data, vertices, tris);
}

