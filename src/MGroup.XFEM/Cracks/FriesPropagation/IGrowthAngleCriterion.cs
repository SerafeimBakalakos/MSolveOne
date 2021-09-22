using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.XFEM.Cracks.FriesPropagation
{
	public interface IGrowthAngleCriterion
	{
		int FindIndexOfPropagationAngle(List<double> trialAngles, List<double> stressesThetaTheta, List<double> stressesRTheta);
	}
}
