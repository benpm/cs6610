#include <surface_net.hpp>

constexpr uint32_t R[] = {1, chunkSize + 1, (chunkSize + 1) * (chunkSize + 1)};
constexpr std::array<uint16_t[3], 6> faceOffsets = {{
    {1,1,1},
    {0,1,1},
    {0,0,1},
    {1,1,1},
    {0,0,1},
    {1,0,1},
}};

// Compute surface vertex positions from data
void meshNet(const Vector<float, chunkCells>& data, std::vector<Vector3f>& vertices, std::vector<uint32_t>& tris) {
    // Contains vertex indices for each cell
    Vector<int, chunkCells> idxBuf;
    idxBuf.fill(-1);

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
        if (occupancy != 0u && occupancy != 8u) {
            idxBuf[flatIdx(x, y, z)] = vertices.size();
            vertices.push_back({(float)x, (float)y, (float)z});
        }
    }

    for (uint16_t z = 0; z < chunkSize - 1; z++)
    for (uint16_t x = 0; x < chunkSize - 1; x++)
    for (uint16_t y = 0; y < chunkSize - 1; y++) {
        int vIdx = idxBuf[flatIdx(x, y, z)];
        if (vIdx >= 0) {
            const uint16_t c[3] = {x + 1u, y + 1u, z + 1u};
            const bool flip = false;//idxBuf[flatIdx(c[0], c[1], c[2])] >= 0;
            
            // Create triangles
            for (uint16_t fID = 0; fID < 3; fID++) { // Face idx
                int vertIDs[6];
                bool validFace = true;
                for (uint16_t j = 0; j < 6; j++) { // Edge idx
                    const uint16_t eID = flip ? 5 - j : j;
                    // Face coordinates
                    int fc[3];
                    for (uint16_t d = 0; d < 3; d++) {
                        fc[d] = c[d] - faceOffsets[eID][(d + fID) % 3];
                    }
                    vertIDs[eID] = idxBuf[flatIdx(fc[0], fc[1], fc[2])];
                    if (vertIDs[eID] < 0) {
                        validFace = false;
                        break;
                    }
                }
                if (validFace) {
                    for (int id : vertIDs) {
                        tris.push_back(id);
                    }
                }
            }
        }
    }
}

void SurfaceNet::build(const Vector<float, chunkCells>& data) {
    meshNet(data, vertices, tris);
}

