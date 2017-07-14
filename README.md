---
## Udacity's Self-Driving Car Engineer Nanodegree Program
---
# PID Controller Project
### Using Gradient Descent with Error Backpropagation to Update Individual Parameters
---

Parameters have been initially set to those derived in Project's Lesson by Sebastian Thrun using twiddle:

```
Kp = 0.2
Ki = 0.004
kd = 3.0
```

I implemented **backpropagation** in the following function:

```c++
/**
 * @param Kx: Kp, Ki or Kd
 * @param dx: partial derivative for Kp, Ki or Kd respectively
 * @param dE: total delta error over the whole epoch (previous epoch - current epoch)
 */
void PID::adjust(double &Kx, double dx, double dE) {
    double partialDKx = Kx * dx * dE * learnRate_;
    Kx -= partialDKx;
}
```

It computes individual (one of three among `Kp`, `Ki` and `Kd`) parameters' contribution to the total error and 
updates it accordingly.

The function that updates the steering value is defined as:

`f(p_err, i_err, d_err) = -Kp * p_err - Ki * i_err - Kd * d_err`

Since I am tuning `Kp`, `Ki` and `Kd`, to obtain partial derivatives **with respect to** `Kp`, `Ki` and `Kd`,
that function must be viewed as `f(Kp, Ki, Kd)` with the partial derivatives equal to 
`-p_error`, `-i_error` and `-d_error` accordingly.

**The tricky part** is with `i_error`. Since I evaluate cumulative error through some number of iterations (say, `200`),
and each iteration's **CTE** might be both positive or negative, their raw summation (which is `i_error`) doesn't
represent the magnitude of that parameter as positives and negatives cancel each other out. To address the issue, 
I've introduced the new variable (creatively called it `i_e_fabs_`), that accumulates absolute values of **CTE**
throughout the epoch, and use it as a measure of the partial derivative for `Ki`.

[The video](https://youtu.be/PDvEPJj8Nmg) illustrates the training process. 

It can be seen that the **backpropagation** updates are mostly concerned about `Ki`, with `Kp` and `Kd` staying pretty
much unchanged and oscillating around `0.2` and `3.0`, while `Ki` **rapidly drops by one order of magnitude** to some
value around `0.0003` or `3e-4`, having even gone to the negative territory for a brief period of time.

---
##### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Alternatively, there are convenient scripts provided by Tiffany Huang for
[Particle Filter project](https://github.com/antevis/CarND-T2-P3_ParticleFilter): `clean.sh`, `build.sh`.
