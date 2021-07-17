using System;
using System.Collections.Generic;
using System.Text;

//TODO: This belongs in LinearAlgebra project
namespace MGroup.LinearAlgebra.Distributed
{
	public interface ILinearTransformation
	{
		void MultiplyVector(IGlobalVector input, IGlobalVector output);
	}
}
