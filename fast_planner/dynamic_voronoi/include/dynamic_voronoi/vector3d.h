#ifndef VECTOR3D
#define VECTOR3D
namespace fast_planner {
    class Vector3D {
        public:
            inline Vector3D(const float x=0, const float y=0, const float z=0) {this->x=x;this->y=y;this->z=z;}
            inline Vector3D operator * (const float k) const {return Vector3D(x*k, y*k, z*k);}
            inline Vector3D operator / (const float k) const {return Vector3D(x/k, y/k, z/k);}
            inline Vector3D operator + (const Vector3D& b) const {return Vector3D(x+b.x, y+b.y, z+b.z);}
            inline Vector3D operator - (const Vector3D& b) const {return Vector3D(x-b.x, y-b.y, z-b.z);}
            inline Vector3D operator - () const  {return Vector3D(-x, -y, -z);}
            /// a convenience method to print a vector
            friend std::ostream& operator<<(std::ostream& os, const Vector3D& b) {os << "(" << b.x << "|" << b.y << b.z << ")"; return os; }
            /// a method to calculate the length of the vector
            float length() const { return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2)); }
            /// a method to calculate the length of the vector
            float sqlength() const { return x*x + y*y + z*z; }
            /// a method to calculate the dot product of two vectors
            float dot(Vector3D b) { return x * b.x + y * b.y + z * b.z; }
            ///a method that returns the orthogonal complement of two vectors
            inline Vector3D ort(Vector3D b) {
                Vector3D a(this->x, this->y, this->z);
                Vector3D c;
                // multiply b by the dot product of this and b then divide it by b's length
                c = a - b * a.dot(b) / b.sqlength();
                return c;
            }
            inline float getX() { return x; }
            inline float getY() { return y; }
            inline float getZ() { return z; }
            //  void setT(float t) { this->t = t; }
            //  float getT() { return t; }
            private:
            /// the x part of the vector
            float x;
            /// the y part of the vector
            float y;
            float z;
            //  /// the theta part for plotting purposes
            //  float t;
    };
    inline Vector3D operator * (double k, const Vector3D& b)
    {
        return (b*k);
    }
}
#endif