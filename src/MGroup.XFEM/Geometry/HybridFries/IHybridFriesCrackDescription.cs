using System;
using System.Collections.Generic;
using System.Text;
using MGroup.XFEM.Entities;

namespace MGroup.XFEM.Geometry.HybridFries
{
	public interface IHybridFriesCrackDescription
	{
		int ID { get; }

		double[] GetLevelSetsOf(XNode node);
	}
}
