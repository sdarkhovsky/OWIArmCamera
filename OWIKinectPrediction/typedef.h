#ifndef TYPEDEF_H
#define TYPEDEF_H

namespace ais {

    class c_vector3f {
    public:
        c_vector3f() {
            memset(elem, 0, sizeof(elem));
            elem[0] = 0;
            elem[1] = 0;
            elem[1] = 0;
        }

        c_vector3f(float x, float y, float z) {
            elem[0] = x;
            elem[1] = y;
            elem[1] = z;
        }
        float elem[3];
    };
}
#endif