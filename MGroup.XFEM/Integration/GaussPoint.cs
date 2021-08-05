using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Integration
{
	/// <summary>
	/// Integration point (coordinates & weight) defined in the 1D, 2D or 3D natural coordinate system of a finite element.
	/// </summary>
	public class GaussPoint
	{
		/// <summary>
		/// Creates a new instance of <see cref="GaussPoint"/>
		/// </summary>
		public GaussPoint(double[] coordinates, double weight)
		{
			this.Coordinates = coordinates;
			this.Weight = weight;
		}

		public double[] Coordinates { get; }

		/// <summary>
		/// The weight factor of this integration point.
		/// </summary>
		public double Weight { get; }
	}
}
