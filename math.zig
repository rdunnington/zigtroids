const std = @import("std");

pub const Vector2 = struct {
    x: f32 = 0,
    y: f32 = 0,

    pub fn toVector3(v: Vector2) Vector3 {
        return Vector3{
            .x = v.x,
            .y = v.y,
            .z = 0.0,
        };
    }
};

pub const Vector3 = struct {
    x: f32 = 0,
    y: f32 = 0,
    z: f32 = 0,

    pub const right = Vector3{ .x = 1, .y = 0, .z = 0 };
    pub const up = Vector3{ .x = 0, .y = 1, .z = 0 };
    pub const forward = Vector3{ .x = 0, .y = 0, .z = 1 };

    pub fn lengthSq(v: Vector3) f32 {
        return v.x * v.x +
            v.y * v.y +
            v.z * v.z;
    }

    pub fn length(v: Vector3) f32 {
        return std.math.sqrt(v.lengthSq());
    }

    pub fn normalize(v: Vector3) Vector3 {
        var len = v.length();
        return Vector3{
            .x = v.x / len,
            .y = v.y / len,
            .z = v.z / len,
        };
    }

    pub fn scale(v: Vector3, s: f32) Vector3 {
        return Vector3{
            .x = v.x * s,
            .y = v.y * s,
            .z = v.z * s,
        };
    }

    pub fn add(a: Vector3, b: Vector3) Vector3 {
        return Vector3{
            .x = a.x + b.x,
            .y = a.y + b.y,
            .z = a.z + b.z,
        };
    }

    pub fn sub(a: Vector3, b: Vector3) Vector3 {
        return Vector3{
            .x = a.x - b.x,
            .y = a.y - b.y,
            .z = a.z - b.z,
        };
    }

    pub fn mul(a: Vector3, b: Vector3) Vector3 {
        return Vector3{
            .x = a.x * b.x,
            .y = a.y * b.y,
            .z = a.z * b.z,
        };
    }

    pub fn div(a: Vector3, b: Vector3) Vector3 {
        return Vector3{
            .x = a.x / b.x,
            .y = a.y / b.y,
            .z = a.z / b.z,
        };
    }

    pub fn dot(a: Vector3, b: Vector3) f32 {
        return a.x * b.x +
            a.y * b.y +
            a.z * b.z;
    }

    pub fn cross(a: Vector3, b: Vector3) Vector3 {
        return Vector3{
            .x = a.y * b.z - a.z * b.y,
            .y = a.z * b.x - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    pub fn asVec4(v: Vector3) Vector4 {
        return Vector4{ .x = v.x, .y = v.y, .z = v.z, .w = 0 };
    }

    pub fn rotateX(v: Vector3, angle: f32) Vector3 {
        const transform = Mat4x4.axisAngle(right, angle);
        return transform.mulv3(v);
    }

    pub fn rotateY(v: Vector3, angle: f32) Vector3 {
        const transform = Mat4x4.axisAngle(up, angle);
        return transform.mulv3(v);
    }

    pub fn rotateZ(v: Vector3, angle: f32) Vector3 {
        const transform = Mat4x4.axisAngle(forward, angle);
        return transform.mulv3(v);
    }

    pub fn log(v: Vector3) void {
        std.debug.warn("Vector3: {:.2} {:.2} {:.2}\n", v.x, v.y, v.z);
    }
};

pub const Vector4 = struct {
    x: f32 = 0,
    y: f32 = 0,
    z: f32 = 0,
    w: f32 = 0,

    pub fn asVec3(v: Vector4) Vector3 {
        return Vector3{ .x = v.x, .y = v.y, .z = v.z };
    }

    pub fn scale(v: Vector4, s: f32) Vector4 {
        return Vector4{
            .x = v.x * s,
            .y = v.y * s,
            .z = v.z * s,
            .w = v.w * s,
        };
    }

    pub fn add(a: Vector4, b: Vector4) Vector4 {
        return Vector4{
            .x = a.x + b.x,
            .y = a.y + b.y,
            .z = a.z + b.z,
            .w = a.w + b.w,
        };
    }

    pub fn dot(a: Vector4, b: Vector4) f32 {
        return a.x * b.x +
            a.y * b.y +
            a.z * b.z +
            a.w * b.w;
    }
};

// pub const Mat3x3 = struct {
//     cols = [3]Vector3{
//         Vector3{ .x = 1, .y = 0, .z = 0 },
//         Vector3{ .x = 0, .y = 1, .z = 0 },
//         Vector3{ .x = 0, .y = 0, .z = 1 },
//     },

//     pub fn identity() Mat3x3 {
//         return Mat3x3{};
//     }

//     pub fn transpose(m: Mat3x3) Mat3x3 {
//         return Mat3x3{
//             .c1 = Vector3{ .x = m.c1.x, .y = m.c2.x, .z = m.c3.x },
//             .c2 = Vector3{ .x = m.c1.y, .y = m.c2.y, .z = m.c3.y },
//             .c3 = Vector3{ .x = m.c1.z, .y = m.c2.z, .z = m.c3.z },
//         };
//     }

//     pub fn mul(m: Mat3x3, v: Vector3) Vector3 {
//         return Vector3{
//             .x = Vector3.dot(t.c1, v),
//             .y = Vector3.dot(t.c2, v),
//             .z = Vector3.dot(t.c3, v),
//         };
//     }

//     pub fn mul(m1: Mat3x3, m2: Mat3x3) Mat3x3 {
//         var t1 = Mat3x3.transpose(m1);

//         return Mat3x3{
//             .c1 = t1.mul(m2.c1),
//             .c2 = t1.mul(m2.c2),
//             .c3 = t1.mul(m2.c3),
//             .c4 = t1.mul(m2.c4),
//         };
//     }
// };

pub const Mat4x4 = struct {
    cols: [4]Vector4 = [4]Vector4{
        Vector4{ .x = 1, .y = 0, .z = 0, .w = 0 },
        Vector4{ .x = 0, .y = 1, .z = 0, .w = 0 },
        Vector4{ .x = 0, .y = 0, .z = 1, .w = 0 },
        Vector4{ .x = 0, .y = 0, .z = 0, .w = 1 },
    },

    pub fn identity() Mat4x4 {
        return Mat4x4{};
    }

    pub fn transpose(m: Mat4x4) Mat4x4 {
        return Mat4x4{
            .cols = [4]Vector4{
                Vector4{ .x = m.c1.x, .y = m.c2.x, .z = m.c3.x },
                Vector4{ .x = m.c1.y, .y = m.c2.y, .z = m.c3.y },
                Vector4{ .x = m.c1.z, .y = m.c2.z, .z = m.c3.z },
                Vector4{ .x = m.c1.z, .y = m.c2.z, .z = m.c3.z },
            },
        };
    }

    pub fn mul(m1: Mat4x4, m2: Mat4x4) Mat4x4 {
        var t1 = Mat4x4.transpose(m1);
        return Mat4x4{
            .cols = [4]Vector{
                t1.mul(m2.cols[0]),
                t1.mul(m2.cols[1]),
                t1.mul(m2.cols[2]),
                t1.mul(m2.cols[3]),
            },
        };
    }

    pub fn mulv3(m: Mat4x4, v: Vector3) Vector3 {
        return m.mulv4(v.asVec4()).asVec3();
    }

    pub fn mulv4(m: Mat4x4, v: Vector4) Vector4 {
        return Vector4{
            .x = m.cols[0].dot(v),
            .y = m.cols[1].dot(v),
            .z = m.cols[2].dot(v),
            .w = m.cols[3].dot(v),
        };
    }

    pub fn axisAngle(axis: Vector3, angle: f32) Mat4x4 {
        const c = std.math.cos(angle);
        const k = 1 - c;
        const s = std.math.sin(angle);

        const x = axis.x;
        const y = axis.y;
        const z = axis.z;

        const m00: f32 = k * x * x + c;
        const m10: f32 = k * x * y + z * s;
        const m20: f32 = k * x * z - y * s;

        const m01: f32 = k * x * y - z * s;
        const m11: f32 = k * y * y + c;
        const m21: f32 = k * y * z + x * s;

        const m02: f32 = k * x * z + y * s;
        const m12: f32 = k * y * z - x * s;
        const m22: f32 = k * z * z + c;

        // zig fmt: off
        return Mat4x4{
            .cols = [4]Vector4{
                Vector4{ .x = m00, .y = m10, .z = m20, .w = 0 },
                Vector4{ .x = m01, .y = m11, .z = m21, .w = 0 },
                Vector4{ .x = m02, .y = m12, .z = m22, .w = 0 },
                Vector4{ .x = 0, .y = 0, .z = 0, .w = 1 },
            },
        };
        // zig fmt: on
    }
};
