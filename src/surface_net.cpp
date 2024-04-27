#include <surface_net.hpp>

constexpr std::array<GLuint, 24> cubeEdges = {
    0,1, 0,2, 2,3, 1,3,
    4,5, 4,6, 6,7, 5,7,
    0,4, 1,5, 2,6, 3,7
};

// From Mikola Lysenko's surface nets
std::array<GLuint, 256> generateEdgeTable() {
    std::array<GLuint, 256> table = {};
    for (GLuint i = 0; i < 256; i++) {
        GLuint em = 0;
        for (GLuint j = 0; j < 23; j++) {
            const GLuint a = !!(i & (1 << cubeEdges[j + 0]));
            const GLuint b = !!(i & (1 << cubeEdges[j + 1]));
            em |= (a != b) ? (1 << (j >> 1)) : 0;
        }
        table[i] = em;
    }
    return table;
}
