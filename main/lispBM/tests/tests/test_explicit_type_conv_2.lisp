
(define r1 (and (eq (to-i64 1u) 1i64)
                (eq (to-i64 1)  1i64)
                (eq (to-i64 1i32) 1i64)
                (eq (to-i64 1u32) 1i64)
                (eq (to-i64 1i64) 1i64)
                (eq (to-i64 1u64) 1i64)
                (eq (to-i64 1.0f32) 1i64)
                (eq (to-i64 1.0f64) 1i64)))

(define r2 (and (eq (to-u64 1u) 1u64)
                (eq (to-u64 1)  1u64)
                (eq (to-u64 1i32) 1u64)
                (eq (to-u64 1u32) 1u64)
                (eq (to-u64 1i64) 1u64)
                (eq (to-u64 1u64) 1u64)
                (eq (to-u64 1.0f32) 1u64)
                (eq (to-u64 1.0f64) 1u64)))

(define r3 (and (eq (to-float 1u) 1.0f32)
                (eq (to-float 1)  1.0f32)
                (eq (to-float 1i32) 1.0f32)
                (eq (to-float 1u32) 1.0f32)
                (eq (to-float 1i64) 1.0f32)
                (eq (to-float 1u64) 1.0f32)
                (eq (to-float 1.0f32) 1.0f32)
                (eq (to-float 1.0f64) 1.0f32)))

(define r4 (and (eq (to-double 1u) 1.0f64)
                (eq (to-double 1)  1.0f64)
                (eq (to-double 1i32) 1.0f64)
                (eq (to-double 1u32) 1.0f64)
                (eq (to-double 1i64) 1.0f64)
                (eq (to-double 1u64) 1.0f64)
                (eq (to-double 1.0f32) 1.0f64)
                (eq (to-double 1.0f64) 1.0f64)))

(check (and r1 r2 r3 r4)) 
