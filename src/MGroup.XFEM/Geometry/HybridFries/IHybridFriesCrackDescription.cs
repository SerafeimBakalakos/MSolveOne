using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;
using MGroup.XFEM.Geometry.LSM;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface IHybridFriesCrackDescription : IImplicitCrackGeometry
	{
		int ID { get; }

		double[] GetTripleLevelSetsOf(XNode node);
	}
}
