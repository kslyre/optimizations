#include "derivable.h"

Derivable::Derivable()
{

}

Derivable Derivable::operator+(const Derivable &f2) const
{
    return Derivable(val + f2.val, deriv + f2.deriv);
}

Derivable Derivable::operator-(const Derivable &f2) const
{
    return Derivable(val - f2.val, deriv - f2.deriv);
}

Derivable Derivable::operator*(const Derivable &f2) const
{
    return Derivable(val * f2.val, deriv * f2.val + val * f2.deriv);
}

Derivable Derivable::operator*(const float &f2) const
{
    return Derivable(val * f2, deriv * f2);
}

Derivable Derivable::operator/(const Derivable &f2) const
{
    return Derivable(val / f2.val, (deriv * f2.val - val * f2.deriv) / f2.val / f2.val);
}

Derivable Derivable::cos(Derivable f)
{
    return Derivable(qCos(f.val), -qSin(f.val)*f.deriv);
}

Derivable Derivable::sin(Derivable f)
{
    return Derivable(qSin(f.val), qCos(f.val)*f.deriv);
}

Derivable Derivable::pow(Derivable f, double val)
{
    return Derivable(qPow(f.val, val), val*qPow(f.val, val-1.f)*f.deriv);
}
