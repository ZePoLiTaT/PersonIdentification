#pragma once
#include "vtkFiltersModelingModule.h" // For export macro
#include "vtkGraphGeodesicPath.h"

class vtkDijkstraGraphInternals;
class vtkIdList;

class DijkstraGraphGeodesicPath :
	public vtkGraphGeodesicPath
{
public:

	// Description:
	// Instantiate the class
	static DijkstraGraphGeodesicPath *New();

	// Description:
	// Standard methids for printing and determining type information.
	vtkTypeMacro(DijkstraGraphGeodesicPath, vtkGraphGeodesicPath);
	void PrintSelf(ostream& os, vtkIndent indent);

	// Description:
	// The vertex ids (of the input polydata) on the shortest path
	vtkGetObjectMacro(IdList, vtkIdList);

	// Description:
	// Stop when the end vertex is reached
	// or calculate shortest path to all vertices
	vtkSetMacro(StopWhenEndReached, int);
	vtkGetMacro(StopWhenEndReached, int);
	vtkBooleanMacro(StopWhenEndReached, int);

	// Description:
	// Use scalar values in the edge weight (experimental)
	vtkSetMacro(UseScalarWeights, int);
	vtkGetMacro(UseScalarWeights, int);
	vtkBooleanMacro(UseScalarWeights, int);

	// Description:
	// Use the input point to repel the path by assigning high costs.
	vtkSetMacro(RepelPathFromVertices, int);
	vtkGetMacro(RepelPathFromVertices, int);
	vtkBooleanMacro(RepelPathFromVertices, int);

	// Description:
	// Specify vtkPoints to use to repel the path from.
	virtual void SetRepelVertices(vtkPoints*);
	vtkGetObjectMacro(RepelVertices, vtkPoints);

	//Description:
	//Fill the array with the cumulative weights.
	virtual void GetCumulativeWeights(vtkDoubleArray *weights);
	//Call after algorithm update to check whether the path was found.
	bool checkPathSuccess();
protected:
	DijkstraGraphGeodesicPath();
	~DijkstraGraphGeodesicPath();

	virtual int RequestData(vtkInformation *, vtkInformationVector **,
		vtkInformationVector *);

	// Build a graph description of the input.
	virtual void BuildAdjacency(vtkDataSet *inData);

	vtkTimeStamp AdjacencyBuildTime;

	// The fixed cost going from vertex u to v.
	virtual double CalculateStaticEdgeCost(vtkDataSet *inData, vtkIdType u, vtkIdType v);

	// The cost going from vertex u to v that may depend on one or more vertices
	//that precede u.
	virtual double CalculateDynamicEdgeCost(vtkDataSet *, vtkIdType, vtkIdType)
	{
		return 0.0;
	}

	void Initialize(vtkDataSet *inData);

	void Reset();

	// Calculate shortest path from vertex startv to vertex endv.
	virtual void ShortestPath(vtkDataSet *inData, int startv, int endv);

	// Relax edge u,v with weight w.
	void Relax(const int& u, const int& v, const double& w);

	// Backtrace the shortest path
	void TraceShortestPath(vtkDataSet* inData, vtkPolyData* outPoly,
		vtkIdType startv, vtkIdType endv);

	// The number of vertices.
	int NumberOfVertices;

	// The vertex ids on the shortest path.
	vtkIdList *IdList;

	//Internalized STL containers.
	vtkDijkstraGraphInternals *Internals;

	int StopWhenEndReached;
	int UseScalarWeights;
	int RepelPathFromVertices;

	vtkPoints* RepelVertices;

	//Flag to test whether the path was found
	bool pathSuccess;

private:
	DijkstraGraphGeodesicPath(const DijkstraGraphGeodesicPath&);  // Not implemented.
	void operator=(const DijkstraGraphGeodesicPath&);  // Not implemented.

};
