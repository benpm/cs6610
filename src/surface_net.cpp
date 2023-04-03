#include <surface_net.hpp>

constexpr uint32_t R[] = {1, chunkSize + 1, (chunkSize + 1) * (chunkSize + 1)};

// Compute surface vertex positions from data
void meshNet(const Vector<float, chunkCells>& data, std::vector<Vector3f>& vertices, std::vector<uint32_t>& tris) {
    // X/Y plane used to connect vertices
    Vector<int, chunkCells> buf;
    buf.fill(0);

    // Sweep through volume placing vertices
    for (uint16_t z = 0; z < chunkSize - 1u; z++) {
        for (uint16_t x = 0; x < chunkSize - 1u; x++)
        for (uint16_t y = 0; y < chunkSize - 1u; y++) {
            Vector3f v((float)x, (float)y, (float)z);
            uint16_t occupancy = 0u;
            for (uint16_t i = 0; i < 2; i++)
            for (uint16_t j = 0; j < 2; j++)
            for (uint16_t k = 0; k < 2; k++) {
                if (data[flatIdx(x+i, y+j, z+k)] > 0.0f) {
                    occupancy += 1u;
                }
            }
            if (occupancy != 0u && occupancy != 8u) {
                const uint16_t m = flatIdx(x + 1, y + 1, z + 1);
                buf[m] = vertices.size();
                vertices.push_back(v);
                
                // Create triangles by connecting with previous plane
                const uint16_t c[] = {x, y, z};
                for (uint16_t i = 0; i < 3; i++) {
                    // YZ, ZX, XY
                    const uint16_t iu = (i + 1) % 3, iv = (i + 2) % 3;
                    if (c[iu] == 0 || c[iv] == 0) {
                        continue;
                    }
                    const uint16_t du = R[iu], dv = R[iv];
                    tris.push_back(buf[m]);
                    tris.push_back(buf[m - du]);
                    tris.push_back(buf[m - du - dv]);
                    tris.push_back(buf[m]);
                    tris.push_back(buf[m - du - dv]);
                    tris.push_back(buf[m - dv]);
                }
            }
        }
    }
}

void SurfaceNet::build(const Vector<float, chunkCells>& data) {
    meshNet(data, vertices, tris);
}

