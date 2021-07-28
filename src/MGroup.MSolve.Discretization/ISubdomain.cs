using System.Collections.Generic;
using MGroup.MSolve.DataStructures;

//TODO: tidy up the methods that concern material state
namespace MGroup.MSolve.Discretization
{
	public interface ISubdomain
	{
		int ID { get; }

		bool LinearSystemModified { get; set; }

		IEnumerable<IElement> EnumerateElements();

		IEnumerable<INode> EnumerateNodes();

		int GetMultiplicityOfNode(int nodeID);

		void ResetMaterialsModifiedProperty();

		void SaveMaterialState();
	}
}
