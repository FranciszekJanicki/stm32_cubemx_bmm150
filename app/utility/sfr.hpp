#ifndef SFR_HPP
#define SFR_HPP

#include "matrix.hpp"

namespace utility {

    template <std::floating_point T, std::size_t nX, std::size_t nY = 1UL, std::size_t nU = nY>
    struct SFR {
    public:
        template <std::size_t N, std::size_t M>
        using Mtx = Matrix<T, N, M>;

        Mtx<nU, 1UL>
        get_control(this SFR& self, Mtx<nY, 1UL> const& y_ref, Mtx<nY, 1UL> const& y, Mtx<nX, 1UL> const& x, T const dt)
        {
            try {
                auto const e = y_ref - y;
                auto const int_e = utility::integrate(e, std::exchange(self.prev_e, e), dt);
                auto const u = self.Ki * int_e - self.Kx * x;
                return u;
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
        }

        Mtx<nU, 1UL> operator()(this SFR& self, Mtx<nY, 1UL> const& y_ref, Mtx<nX, 1UL> const& x)
        {
            try {
                return self.get_control(y_ref, x);
            }
            catch (std::runtime_error const& error) {
                throw error;
            }
        }

        /* state */
        Mtx<nX, 1UL> x = {};
        T prev_e = {};

        /* state model */
        Mtx<nX, nX> A = {};
        Mtx<nX, nU> B = {};
        Mtx<nY, nX> C = {};
        Mtx<1UL, nU> D = {};

        /* state gain */
        Mtx<nU, nX> Kx = {};
        Mtx<1UL, nY> Ki = {};
    };

}; // namespace utility

#endif // SFR_HPP