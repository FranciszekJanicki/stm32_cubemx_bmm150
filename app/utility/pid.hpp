#ifndef PID_HPP
#define PID_HPP

#include "utility.hpp"
#include <concepts>
#include <utility>

namespace utility {

    template <std::floating_point T>
    struct PID {
        T operator()(this PID& self, T const e, T const dt) noexcept
        {
            return self.get_sat_u(e, dt);
        }

        T get_sat_u(this PID& self, T const e, T const dt) noexcept
        {
            auto const u = self.get_u(e, dt);
            auto const sat_u = std::clamp(u, -self.sat, self.sat);
            self.prev_sat_e = self.sat_e;
            self.sat_e = u - sat_u;

            return sat_u;
        }

        // basic PID parameters
        T kP = {};
        T kI = {};
        T kD = {};
        T tD = {};

        // anti windup parameters
        T kC = {};
        T sat = {};

        // basic PID internal state
        T prev_e = {};
        T int_e = {};
        T dot_e = {};

        // anti windup internal state
        T sat_e = {};
        T prev_sat_e = {};
        T int_sat_e = {};

        bool is_initialized = {};

    private:
        T get_u(this PID& self, T const e, T const dt) noexcept
        {
            if (!self.is_initialized) {
                self.prev_e = e;
                self.prev_sat_e = self.sat_e;
                self.is_initialized = true;
            }

            auto const u = self.get_p(e) + self.get_i(e, dt) + self.get_d(e, dt);
            self.prev_e = e;

            return u;
        }

        T get_p(this PID& self, T const e) noexcept
        {
            return self.kP * e;
        }

        T get_d(this PID& self, T const e, T const dt) noexcept
        {
            self.dot_e = utility::differentiate(e, self.prev_e, dt, self.prev_e, self.tD);

            return self.kD * self.dot_e;
        }

        T get_i(this PID& self, T const e, T const dt) noexcept
        {
            self.int_e += utility::integrate(e, self.prev_e, dt);
            self.int_sat_e += utility::integrate(self.sat_e, self.prev_sat_e, dt);

            return self.kI * self.int_e - self.kC * self.int_sat_e;
        }
    };

}; // namespace utility

#endif // PID_HPP
