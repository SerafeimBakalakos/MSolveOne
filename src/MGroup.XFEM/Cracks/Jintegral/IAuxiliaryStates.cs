using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.XFEM.Cracks.Geometry;

namespace MGroup.XFEM.Cracks.Jintegral
{
    public interface IAuxiliaryStates
    {
        AuxiliaryStatesTensors ComputeTensorsAt(double[] integrationPointPolar, TipJacobians polarJacobians);
    }
}
