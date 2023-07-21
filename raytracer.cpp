#include <optional>
#include <cmath>
#include <cstdio>
#include <vector>
#include <unistd.h>

const double EPSILON = 1e-9;

struct Vec3 {
    double x, y, z;

    friend Vec3 operator+(Vec3 v, Vec3 w) {
        return {v.x + w.x, v.y + w.y, v.z + w.z};
    }

    friend Vec3 operator-(Vec3 v) {
        return {-v.x, -v.y, -v.z};
    }

    friend Vec3 operator-(Vec3 v, Vec3 w) {
        return {v.x - w.x, v.y - w.y, v.z - w.z};
    }

    friend Vec3 operator*(double k, Vec3 v) {
        return {k * v.x, k * v.y, k * v.z};
    }
    friend Vec3 operator*(Vec3 v, double k) {
        return {v.x * k, v.y * k, v.z * k};
    }

    friend Vec3 operator/(Vec3 v, double k) {
        return {v.x / k, v.y / k, v.z / k};
    }

    friend double operator*(Vec3 v, Vec3 w) {
        return v.x * w.x + v.y * w.y + v.z * w.z;
    }

    friend Vec3 operator^(Vec3 v, Vec3 w) {
        return {v.y * w.z - v.z * w.y,
            v.z * w.x - v.x * w.z,
            v.x * w.y - v.y * w.x};
    }

    double norm() {
        return sqrt(x * x + y * y + z * z);
    }
};
struct Point {
    Vec3 coords;

    friend Point operator+(Point p, Vec3 v) {
        return {p.coords + v};
    }

    friend Point operator-(Point p, Vec3 v) {
        return {p.coords - v};
    }

    friend Vec3 operator-(Point p, Point q) {
        return p.coords - q.coords;
    }

};
struct UVec3 {
    // must be unit
    Vec3 v;

    UVec3(Vec3 v) : v(v / v.norm()) {};
    // be sure!!
    UVec3(double x, double y, double z) : v({x, y, z}) {};

    friend UVec3 operator^(UVec3 v, UVec3 w) {
        Vec3 coords = v.v ^ w.v;
        return UVec3(coords.x, coords.y, coords.z);
    }
};
// const Vec3 ZERO = {0, 0, 0};
// const Vec3 ORIGIN = {ZERO};

struct SphCoords {
    double az, pol;

    UVec3 vec() {
        return UVec3(sin(pol) * cos(az),
            sin(pol) * sin(az),
            cos(pol));
    }
};

struct Ray {
    Point origin;
    UVec3 dir;

    Ray(Point origin, UVec3 dir) : origin(origin), dir(dir) {};
    Ray(Point a, Point b) : origin(a), dir(UVec3(b - a)) {};

    Point at_time(double t) {
        return origin + t * dir.v;
    }
};

struct Frame {
    Point origin;
    UVec3 fwd, rt, up;

    Frame(Point origin, UVec3 fwd, UVec3 rt) :
        origin(origin), fwd(fwd), rt(rt), up(rt ^ fwd) {};
    Frame(Ray r, UVec3 rt) :
        origin(r.origin), fwd(r.dir), rt(rt), up(rt ^ r.dir) {};
};

struct RGB {
    unsigned char r, g, b;

    void write(unsigned char*& buf) {
        buf[0] = r;
        buf[1] = g;
        buf[2] = b;
        buf[3] = 0xFF;
        buf += 4;
    }

    void fwrite(std::FILE* stream) {
        unsigned char buf[4] = {r, g, b, 0xFF};
        std::fwrite(buf, sizeof(unsigned char), 4, stream);
    }
};

struct Hit {
    double t;
    RGB color;
};


class Geometry {
public:
    // Returns the argument for the parameterization of the ray, not the point
    // in space. If there are multiple intersections, returns the one closest
    // to the origin of the ray.
    virtual std::optional<Hit> intersect(Ray) = 0;
};

const RGB plane_color1{0xA0, 0xA0, 0xFF},
      plane_color2{0xA0, 0xA0, 0xA0};
bool even(double x) {return !!abs((int)ceil(x) % 2);}
RGB plane_color(Point p) {
    return
        even(p.coords.x) ^ even(p.coords.y) ^ even(p.coords.z) ?
        plane_color1 : plane_color2;
}
struct Plane : Geometry {
    // the plane is n . [x y z] = k
    // this means that n is the normal to the plane
    Vec3 n;
    double k;

    Plane(Vec3 n, double k) : n(n), k(k) {};

    std::optional<Hit> intersect(Ray r) {
        double numer = k - n * r.origin.coords, denom = n * r.dir.v;
        if (std::abs(denom) < EPSILON) { // ray is parallel to the plane...
            if (std::abs(numer) < EPSILON) { // ...and contained in the plane
                return {{0, plane_color(r.origin)}};
            }
            else { // ...but not contained in the plane
                return {};
            }
        }
        else { // ray is not parallel to the plane
            double t = numer / denom;
            if (t >= 0) return {{t, plane_color(r.at_time(t))}};
            else return {};
        }
    };
};

/*
struct Triangle : Geometry {
    Point a, b, c;

    Triangle(Point a, Point b, Point c) : a(a), b(b), c(c) {};

    std::optional<double> intersect(Ray r) {
        return {};
    };
};
*/

const RGB sphere_color{0xFF, 0xA0, 0xA0};
struct Sphere : Geometry {
    Point center;
    double radius;

    Sphere(Point c, double r) : center(c), radius(r) {};

    std::optional<Hit> intersect(Ray r) {
        Vec3 u = r.dir.v,
             // The equation we want to solve is the same as the one for a
             // sphere centered at ORIGIN with the ray's origin offset by our
             // sphere's actual center.
             o = r.origin - center;
        // coefficients of the quadratic we want to solve
        double a = u * u, // always nonzero bc of where u comes from
               b = 2 * (o * u),
               c = o * o - radius * radius,
               discrim = b * b - 4 * a * c;
        if (discrim < 0) return {};
        double i1 = (-b + sqrt(discrim)) / (2 * a),
               i2 = (-b - sqrt(discrim)) / (2 * a);
        if (i1 <= i2) std::swap(i1, i2);
        if (i1 >= 0) return {{i1, sphere_color}};
        else if (i2 >= 0) return {{i2, sphere_color}};
        else return {};
    };
};

struct Scene : Geometry {
    // TODO should this be a unique_ptr or something?
    std::vector<Geometry*> geoms;

    std::optional<Hit> intersect(Ray r) {
        std::optional<Hit> h = {};
        for (Geometry* g : geoms) {
            auto this_h = g->intersect(r);
            if (this_h && (!h || this_h.value().t < h.value().t)) h = this_h;
        }
        return h;
    }
};

void draw(Geometry& scene, Frame fr,
        double iw, double ih, int pw, int ph,
        unsigned char* buf) {
    for (int y = 0; y < ph; y++) {
        for (int x = 0; x < pw; x++) {
            Vec3 dir = fr.fwd.v +
                fr.up.v * ih * (0.5 - (double)y/(double)ph) +
                fr.rt.v * iw * ((double)x/(double)pw - 0.5);
            Ray r = Ray{fr.origin, UVec3(dir.x, dir.y, dir.z)};
            auto hit = scene.intersect(r);
            RGB pix = hit ? hit.value().color : RGB{0xFF, 0xFF, 0xFF};
            pix.write(buf);
        }
    }
}

int main() {
    // a perfect x=0, y=0, or z=0 plane "z-fights with itself" because of how
    // the color function is written, hence the 0.001
    Plane p{{0, 0, 1}, 0.001};
    Sphere s1{{0, 1.5, 2}, 1};
    Sphere s2{{2, 3, 2.5}, 1};
    Scene sc;
    sc.geoms = {&p, &s1, &s2};

    Point eye_pos{0, -1, 1};
    SphCoords eye_dir{M_PI / 2, M_PI / 2};
    const int w = 480, h = 480;
    unsigned char* buf = (unsigned char*)
        calloc(w * h * 4, sizeof(unsigned char));
    for (;;) {
        Frame fr(eye_pos, eye_dir.vec(), UVec3(1, 0, 0));
        draw(sc, fr, 1, 1, w, h, buf);
        std::fwrite(buf, sizeof(unsigned char), w * h * 4, stdout);
        eye_pos.coords.y -= 0.005;
        eye_pos.coords.z += 0.01;
        eye_dir.pol += 0.001;
    }
    free(buf); // lol
    return 0;
}

