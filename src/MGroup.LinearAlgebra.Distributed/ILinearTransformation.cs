using System;
using System.Collections.Generic;
using System.Text;

namespace MGroup.LinearAlgebra.Distributed
{
	public interface ILinearTransformation
	{
		void MultiplyVector(IGlobalVector input, IGlobalVector output);
	}
}
