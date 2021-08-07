using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MGroup.XFEM.Cracks.Jintegral
{
    public interface ISifCalculator
    {
        double CalculateSif(double interactionIntegral);
    }
}
