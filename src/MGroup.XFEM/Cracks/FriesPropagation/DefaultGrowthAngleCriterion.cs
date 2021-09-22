using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public class DefaultGrowthAngleCriterion : IGrowthAngleCriterion
	{
		private readonly double relativeStressRThetaTolerance;

		public DefaultGrowthAngleCriterion(double relativeStressRThetaTolerance)
		{
			this.relativeStressRThetaTolerance = relativeStressRThetaTolerance;
		}

		public int FindIndexOfPropagationAngle(
			List<double> trialAngles, List<double> stressesThetaTheta, List<double> stressesRTheta)
		{
			double tolerance = CalcZeroTolerance(stressesRTheta);
			int resultIndex = -1;
			double maxStressThetaTheta = double.MinValue;
			for (int i = 0; i < trialAngles.Count; ++i)
			{
				if (Math.Abs(stressesRTheta[i]) <= tolerance)
				{
					if (stressesThetaTheta[i] > maxStressThetaTheta)
					{
						resultIndex = i;
						maxStressThetaTheta = stressesThetaTheta[i];
					}
				}
			}

			if (resultIndex == -1)
			{
				throw new Exception("Cound not find any point with: r-theta stress = 0");
			}
			else
			{
				if (stressesThetaTheta[resultIndex] < 0)
				{
					throw new Exception("Cound not find any point with: (r-theta stress = 0) AND (theta-theta stress > 0)");
				}
				return resultIndex;
			}
		}

		private double CalcZeroTolerance(List<double> stressesRTheta)
		{
			double maxStressRTheta = double.MinValue;
			foreach (double s in stressesRTheta)
			{
				if (s > maxStressRTheta)
				{
					maxStressRTheta = s;
				}
			}
			return relativeStressRThetaTolerance * maxStressRTheta;
		}
	}
}
