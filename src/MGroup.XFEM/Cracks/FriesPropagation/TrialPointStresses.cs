using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	/// <summary>
	/// Stresses and position of trial points around a crack tip.
	/// </summary>
	public class TrialPointStresses
	{

		public TrialPointStresses(double[] crackTip)
		{
			CrackTip = crackTip;
		}

		public List<double> Angles = new List<double>();

		public int CriticalAngleIndex { get; set; } = -1;

		public double[] CrackTip { get; }

		public double GrowthLength { get; set; } = double.NaN;

		public List<double[]> CoordinatesGlobal = new List<double[]>();

		public List<double[]> StressesGlobal = new List<double[]>();

		public List<double> StressesThetaTheta = new List<double>();

		public List<double> StressesRTheta = new List<double>();
	}
}
