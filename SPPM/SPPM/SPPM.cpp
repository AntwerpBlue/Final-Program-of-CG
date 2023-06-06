#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include"bvh.hpp"
int main()
{
    // randomly generate some trangles
    std::vector<Renderable* > objs;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_real_distribution<float> u01(0, 1);

    using v3f = Eigen::Vector3f;
    for (int i = 0; i < 100; ++i)
    {
        v3f iv0(u01(rng), u01(rng), u01(rng));
        v3f iv1(u01(rng), u01(rng), u01(rng));
        v3f iv2(u01(rng), u01(rng), u01(rng));


        auto t = new Triangle(iv0, iv1, iv2, 1, false, true, false, false, 0.0, 1.0, 0.0, v3f(1.0, 1.0, 1.0), 699.f, 1.0f);
        objs.push_back(t);
    }

    BVH bvh(objs);
    Ray r(v3f(0.5, 0.5, 0), v3f(0, 0, 1));
    auto info = bvh.get_first_intersection(r);
    std::cout << "Distance:" << info.distance << "\n"
        << "Position:\n" << info.pos << "\n";


    Ray next = info.obj->get_next_ray(r, info.pos);

    std::cout << "Ray Pos\n" << next.pos << "\nDir\n" << next.dir << "\n";


    Photon emit = info.obj->emit_a_photon();
    std::cout << "Photon Pos\n" << emit.ray.pos << "\nDir\n" << emit.ray.dir << "\nPower\n" << emit.power_rgb << "\n";
}
