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

    friend double operator*(Vec3 v, Vec3 w) {
        return v.x * w.x + v.y * w.y + v.z * w.z;
    }

    friend Vec3 operator^(Vec3 v, Vec3 w) {
        return {v.y * w.z - v.z * w.y,
            v.z * w.x - v.x * w.z,
            v.x * w.y - v.y * w.x};
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
// const Vec3 ZERO = {0, 0, 0};
// const Vec3 ORIGIN = {ZERO};

struct SphCoords {
    double az, pol;

    Vec3 vec() {
        return {sin(pol) * cos(az),
            sin(pol) * sin(az),
            cos(pol)};
    }
};

struct Ray {
    Point origin;
    SphCoords dir;

    Point at_time(double t) {
        return origin + t * dir.vec();
    }
};


class Geometry {
public:
    // Returns the argument for the parameterization of the ray, not the point
    // in space. If there are multiple intersections, returns the one closest
    // to the origin of the ray.
    virtual std::optional<double> intersect(Ray) = 0;
};

struct Plane : Geometry {
    // the plane is n . [x y z] = k
    // this means that n is the normal to the plane
    Vec3 n;
    double k;

    Plane(Vec3 n, double k) : n(n), k(k) {};

    std::optional<double> intersect(Ray r) {
        Vec3 u = r.dir.vec();
        double numer = k - n * r.origin.coords, denom = n * u;
        if (std::abs(denom) < EPSILON) { // ray is parallel to the plane...
            if (std::abs(numer) < EPSILON) { // ...and contained in the plane
                return 0;
            }
            else { // ...but not contained in the plane
                return {};
            }
        }
        else { // ray is not parallel to the plane
            double t = numer / denom;
            if (t >= 0) return t;
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

std::optional<double> min_nonneg(double a, double b) {
    if (a <= b) {
        if (a >= 0) return a;
        else if (b >= 0) return b;
        else return {};
    }
    else {
        if (b >= 0) return b;
        else if (a >= 0) return a;
        else return {};
    }
}

struct Sphere : Geometry {
    Point center;
    double radius;

    Sphere(Point c, double r) : center(c), radius(r) {};

    std::optional<double> intersect(Ray r) {
        Vec3 u = r.dir.vec(),
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
        return min_nonneg(i1, i2);
    };
};

struct Scene : Geometry {
    // TODO should this be a unique_ptr or something?
    std::vector<Geometry*> geoms;

    std::optional<double> intersect(Ray r) {
        std::optional<double> t = {};
        for (Geometry* g : geoms) {
            auto this_t = g->intersect(r);
            if (this_t && (!t || this_t.value() < t.value())) t = this_t;
        }
        return t;
    }
};

void draw(Geometry& scene, Ray r0, double res) {
    for (Ray eye = r0; eye.dir.pol < r0.dir.pol + M_PI; eye.dir.pol += res) {
        for (eye.dir.az = r0.dir.az + M_PI/2;
                eye.dir.az > r0.dir.az - M_PI/2; eye.dir.az -= res / 2) {
            std::putchar(scene.intersect(eye) ? '#' : ' ');
        }
        std::putchar('\n');
    }
}
void clear() {
    printf("\033[2J\033[3J\033[H");
}

int main() {
    Plane p{{0, 0, 1}, 0};
    Sphere s{{0, 1.5, 2}, 1};
    Scene sc;
    sc.geoms = {&p, &s};

    Ray eye{{0, 0, 1}, {M_PI / 2, 0}};
    for (;;) {
        clear();
        draw(sc, eye, 0.03);
        eye.origin.coords.y -= 0.01;
        eye.dir.pol += 0.001;
        usleep(100000);
    }
    /*
    std::optional<double> i = p.intersect(eye);
    if (i) std::printf("%f\n", i.value());
    else std::printf("no intersection\n");
    return 0;
    */
    return 0;
}

