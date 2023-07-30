#include <optional>
#include <cmath>
#include <cstdio>
#include <vector>
#include <unistd.h>
#include <random>

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
struct Mat3 {
    Vec3 col1, col2, col3;

    friend Vec3 operator*(Mat3 m, Vec3 v) {
        return m.col1 * v.x + m.col2 * v.y + m.col3 * v.z;
    }

    friend Mat3 operator*(Mat3 m, Mat3 n) {
        return {m * n.col1, m * n.col2, m * n.col3};
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

    // be sure!!
    UVec3(Vec3 v) : v(v) {}
    UVec3(double x, double y, double z) : v({x, y, z}) {}

    friend UVec3 operator-(UVec3 v) {
        return {-v.v};
    }

    friend double operator*(UVec3 v, UVec3 w) {
        return v.v * w.v;
    }
};
UVec3 normalize(Vec3 v) {return {v / v.norm()};}
const UVec3 X = {1, 0, 0},
      Y = {0, 1, 0},
      Z = {0, 0, 1};
struct OMat3 {
    Mat3 m;

    // be sure!!
    OMat3(UVec3 u, UVec3 v, UVec3 w) : m(Mat3{u.v, v.v, w.v}) {}
    OMat3(Mat3 m) : m(m) {}

    friend Vec3 operator*(OMat3 m, Vec3 v) {
        return m.m * v;
    }

    friend UVec3 operator*(OMat3 m, UVec3 v) {
        return {m.m * v.v};
    }

    friend Mat3 operator*(Mat3 m, OMat3 n) {
        return m * n.m;
    }

    friend Mat3 operator*(OMat3 m, Mat3 n) {
        return m.m * n;
    }

    friend OMat3 operator*(OMat3 m, OMat3 n) {
        return OMat3(m.m * n.m);
    }
};
OMat3 rot_to(UVec3 v) {
    UVec3 axis = Z;
    double ax = std::abs(v.v.x),
           ay = std::abs(v.v.y),
           az = std::abs(v.v.z);
    if (ax <= ay && ax <= az) axis = X;
    else if (ay <= ax && ay <= az) axis = Y;
    // else axis = Z; <- using Z as a default value replaces this
    UVec3 w = normalize(v.v ^ axis.v);
    return {UVec3(w.v ^ v.v), w, v};
}
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

    Ray(Point origin, UVec3 dir) : origin(origin), dir(dir) {}
    Ray(Point a, Point b) : origin(a), dir(normalize(b - a)) {}

    Point at_time(double t) {
        return origin + t * dir.v;
    }
};

struct Frame {
    Point origin;
    UVec3 fwd, rt, up;

    // fwd and rt args must be perpendicular
    Frame(Point origin, UVec3 fwd, UVec3 rt) :
        origin(origin), fwd(fwd), rt(rt), up(UVec3(rt.v ^ fwd.v)) {}
    Frame(Ray r, UVec3 rt) :
        origin(r.origin), fwd(r.dir), rt(rt), up(UVec3(rt.v ^ r.dir.v)) {}
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


class Geometry {
public:
    // Returns the argument for the parameterization of the ray, not the point
    // in space. If there are multiple intersections, returns the one closest
    // to the origin of the ray.
    virtual std::optional<double> intersect(Ray) = 0;
    virtual UVec3 normal(Point) = 0;
    virtual double brdf(Point, UVec3, UVec3, double) = 0;
    virtual RGB color(Point) = 0;
};

const RGB plane_color1{0xA0, 0xA0, 0xFF},
      plane_color2{0xA0, 0xA0, 0xA0};
bool even(double x) {return !!abs((int)ceil(x) % 2);}
struct Plane : Geometry {
    // the plane is n . [x y z] = k
    // this means that n is the normal to the plane
    Vec3 w;
    double k;
    UVec3 n;

    Plane(Vec3 w, double k) : w(w), k(k), n(normalize(w)) {}

    std::optional<double> intersect(Ray r) {
        double numer = k - w * r.origin.coords, denom = w * r.dir.v;
        if (std::abs(denom) < EPSILON) { // ray is parallel to the plane...
            if (std::abs(numer) < EPSILON) { // ...and contained in the plane
                return {0};
            }
            else { // ...but not contained in the plane
                return {};
            }
        }
        else { // ray is not parallel to the plane
            double t = numer / denom;
            if (t >= 0) return {t};
            else return {};
        }
    }

    UVec3 normal(Point p) {return n;}

    double brdf(Point p, UVec3 dir_i, UVec3 dir_o, double freq) {
        return 0.5/M_PI;
    }

    RGB color(Point p) {
        return
            even(p.coords.x) ^ even(p.coords.y) ^ even(p.coords.z) ?
            plane_color1 : plane_color2;
    }
};

/*
struct Triangle : Geometry {
    Point a, b, c;

    Triangle(Point a, Point b, Point c) : a(a), b(b), c(c) {}

    std::optional<double> intersect(Ray r) {
        return {};
    }
};
*/

const RGB sphere_color{0xFF, 0xA0, 0xA0};
struct Sphere : Geometry {
    Point center;
    double radius;

    Sphere(Point c, double r) : center(c), radius(r) {}

    std::optional<double> intersect(Ray r) {
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
        if (i1 >= 0) return {i1};
        else if (i2 >= 0) return {i2};
        else return {};
    }

    UVec3 normal(Point p) {return normalize(p - center);}

    double brdf(Point p, UVec3 dir_i, UVec3 dir_o, double freq) {
        return 0.5/M_PI;
    }

    RGB color(Point p) {
        return sphere_color;
    }
};

struct PointOnScene {
    Point p;
    Geometry* g;
};
struct Scene {
    // TODO should this be a unique_ptr or something?
    std::vector<Geometry*> geoms;
    std::ranlux48_base gen;
    std::uniform_real_distribution<double> z_distr;
    std::uniform_real_distribution<double> az_distr;

    Scene(std::vector<Geometry*> geoms) : geoms(geoms) {
        std::random_device rd;
        gen = std::ranlux48_base(rd());
        z_distr = std::uniform_real_distribution<double>(0, 1);
        az_distr = std::uniform_real_distribution<double>(0, 2 * M_PI);
    }

    std::optional<PointOnScene> intersect(Ray r) {
        std::optional<double> t = {};
        std::optional<Geometry*> g = {};
        for (Geometry* this_g : geoms) {
            auto this_t = this_g->intersect(r);
            if (this_t && (!t || this_t.value() < t.value())) {
                t = this_t;
                g = this_g;
            }
        }
        if (!t) return {};
        else return {{r.at_time(t.value()), g.value()}};
    }

    // implying i remember what spectral radiance is
    double skybox_spectral_radiance(double freq) {
        return 1;
    }

    // from PBR by way of glowcoil
    UVec3 sample_north_hemi() {
        double z = z_distr(gen), az = az_distr(gen);
        return UVec3(sqrt(1 - z * z) * cos(az),
            sqrt(1 - z * z) * sin(az),
            z);
    }

    UVec3 sample_hemi(UVec3 apex) {
        return rot_to(apex) * sample_north_hemi();
    }

    const double NUDGE = 0.00001;
    const int SAMPLES = 100;
    const int MAX_BOUNCES = 5;
    double bounce(Point p, UVec3 dir_i, double freq) {
        double accum = 1.0;
        for (int i = 0; i < MAX_BOUNCES && accum > EPSILON; i++) {
            std::optional<PointOnScene> hit = intersect(Ray(p, dir_i));
            if (!hit) {
                accum *= skybox_spectral_radiance(freq);
                break;
            }
            PointOnScene next_p = hit.value();
            UVec3 dir_o = -dir_i;
            UVec3 n = next_p.g->normal(next_p.p);
            // bounce out from here to avoid self-collision
            p = next_p.p + NUDGE * n.v;
            // no emitting surfaces atm, so just the integrand
            dir_i = sample_hemi(n);
            accum *= 2 * M_PI *
                next_p.g->brdf(next_p.p, dir_i, dir_o, freq) * (dir_i * n);
        }
        return accum;
    }
    double L_i(Point p, UVec3 dir_i, double freq) {
        double sum = 0;
        for (int i = 0; i < SAMPLES; i++) sum += bounce(p, dir_i, freq);
        return sum / (double)SAMPLES;
    }
};

void draw_bounce(Scene& scene, Frame fr,
        double iw, double ih, int pw, int ph,
        unsigned char* buf) {
    for (int y = 0; y < ph; y++) {
        for (int x = 0; x < pw; x++) {
            Vec3 dir = fr.fwd.v +
                fr.up.v * ih * (0.5 - (double)y/(double)ph) +
                fr.rt.v * iw * ((double)x/(double)pw - 0.5);
            double rad = scene.L_i(fr.origin, normalize(dir), 1);
            unsigned char val = (unsigned char)(0xFF * std::min(rad, 1.0));
            RGB pix = RGB{val, val, val};
            pix.write(buf);
        }
    }
}

void draw(Scene& scene, Frame fr,
        double iw, double ih, int pw, int ph,
        unsigned char* buf) {
    for (int y = 0; y < ph; y++) {
        for (int x = 0; x < pw; x++) {
            Vec3 dir = fr.fwd.v +
                fr.up.v * ih * (0.5 - (double)y/(double)ph) +
                fr.rt.v * iw * ((double)x/(double)pw - 0.5);
            Ray r = Ray{fr.origin, normalize(dir)};
            auto hit = scene.intersect(r);
            RGB pix = hit ?
                hit.value().g->color(hit.value().p) : RGB{0xFF, 0xFF, 0xFF};
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
    Scene sc({&p, &s1, &s2});

    Point eye_pos{1, -2, 2};
    SphCoords eye_dir{M_PI / 2, M_PI / 2};
    const int w = 480, h = 480;
    unsigned char* buf = (unsigned char*)
        calloc(w * h * 4, sizeof(unsigned char));
    for (;;) {
        Frame fr(eye_pos, eye_dir.vec(), UVec3(1, 0, 0));
        draw_bounce(sc, fr, 1, 1, w, h, buf);
        std::fwrite(buf, sizeof(unsigned char), w * h * 4, stdout);
        eye_pos.coords.y -= 0.005;
        eye_pos.coords.z += 0.01;
        eye_dir.pol += 0.001;
    }
    free(buf); // lol
    return 0;
}

