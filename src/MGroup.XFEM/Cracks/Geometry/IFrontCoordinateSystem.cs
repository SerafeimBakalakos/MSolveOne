using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.Primitives;

namespace MGroup.XFEM.Cracks.Geometry
{
	public interface IFrontCoordinateSystem
	{
		double[] MapPointToLocalPolar(XNode node);

		double[] MapPointToLocalPolar(XPoint point);

		double[] TransformScalarFieldDerivativesLocalPolarToGlobalCartesian(XPoint point, double[] gradientPolar);
	}
}
