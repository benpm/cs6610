#include <surface_net.hpp>

const Vector<uint16_t, 24u> computeCubeVertexIDs() {
    Vector<uint16_t, 24u> cubeVertexIDs;
    size_t k = 0;
    for (uint16_t i = 0; i < 8; i++) {
        for (uint16_t j = 1; j <= 4; j<<=1) {
            const uint16_t p = i^j;
            if (i <= k) {
                cubeVertexIDs[k++] = i;
                cubeVertexIDs[k++] = p;
            }
        }
    }
    return cubeVertexIDs;
}

const std::bitset<256> computeIntersectTable() {
    const Vector<uint16_t, 24u> cubeVertexIDs = computeCubeVertexIDs();

    std::bitset<256> table;
    for(uint16_t i = 0; i < 256u; i++) {
        bool em = false;
        for (uint16_t j = 0; j < 24u; j += 2u) {
            const uint16_t a = i & (1 << cubeVertexIDs[j]);
            const uint16_t b = i & (1 << cubeVertexIDs[j + 1]);
            em |= (a != b) * (1 << (j >> 1));
        }
        table[i] = em;
    }
    return table;
}

const Vector<uint16_t, 24u> SurfaceNet::cubeVertexIDs = computeCubeVertexIDs();
const std::bitset<256> SurfaceNet::intersectTable = computeIntersectTable();

inline size_t flatIdx(uint16_t x, uint16_t y, uint16_t z) {
    return z * (chunkSize * chunkSize) + y * chunkSize + x;
}

// Compute surface vertex positions from data
void meshNet(const std::array<float, chunkCells>& data, std::vector<Vector3f>& vertices, std::vector<uint32_t>& tris) {
    // X/Y plane used to connect vertices
    Matrix<int, chunkSize, chunkSize> back(-1);
    Matrix<int, chunkSize, chunkSize> front(-1);

    // Sweep through volume placing vertices
    for (uint16_t z = 0; z < chunkSize - 1u; z++) {
        for (uint16_t x = 0; x < chunkSize - 1u; x++)
        for (uint16_t y = 0; y < chunkSize - 1u; y++) {
            Vector3f v((float)x, (float)y, (float)z);
            uint16_t occupancy = 0u;
            for (uint16_t i = 0; i < 2; i++)
            for (uint16_t j = 0; j < 2; j++)
            for (uint16_t k = 0; k < 2; k++) {
                if (data[flatIdx(x+i, y+j, z+k)] > 0.5f) {
                    occupancy += 1u;
                }
            }
            if (occupancy != 0u && occupancy != 8u) {
                front(x, y) = vertices.size();
                vertices.push_back(v);
                
                // Create triangles by connecting with previous plane
                if (x > 0 && y > 0) {
                    if (back(x-1, y-1) != -1 && back(x-1, y) != -1) {
                        tris.push_back(back(x-1, y-1));
                        tris.push_back(front(x, y));
                        tris.push_back(back(x-1, y));
                        tris.push_back(back(x-1, y));
                        tris.push_back(front(x, y));
                        tris.push_back(front(x, y-1));
                    }
                }
            }
        }
        back.swap(front);
        front.fill(-1);
    }
}