package main

import (
	"fmt"
	"github.com/gofiber/fiber/v2"
	"github.com/sirupsen/logrus"
	"reflect"
	"sinectis-graphs/Algorith"
	"sinectis-graphs/Types"
)

type Request struct {
	Vertex   string      `json:"vertex"`
	Start    string      `json:"start"`
	End      string      `json:"end"`
	SubGraph interface{} `json:"subGraph"`
}

type GraphResponse struct {
	Graph             Types.Graph              `json:"graph"`
	IncidenceMatrix   Types.IncidenceMatrix    `json:"incidenceMatrix"`
	IncidenceFunction string                   `json:"incidenceFunction"`
	VertexList        string                   `json:"vertexList"`
	DegreeCounts      Types.DegreeCountsString `json:"degreeCounts"`
	Html              Types.Html               `json:"html"`
	SimplePath        string                   `json:"simplePath"`
	SubGraph          []Types.Graph            `json:"subGraph"`
}

func startA(r Request) GraphResponse {

	// Extraer las funciones de incidencia

	// Crear un slice de aristas y un slice de nodos
	edges := make(Types.Edges, 0)
	nodes := make(Types.Nodes, 0)

	nodes, edges = Algorith.FilterInput(r.Vertex)

	//// Encuentra un camino simple entre las ciudades
	simplePath := Algorith.GetSimplePath(r.Start, r.End, edges)

	// Eliminar duplicados de nodos y ordenar alfabéticamente
	nodes = nodes.UniqueStringsAndSort()

	// Crear matriz de incidencia
	incidenceMatrix := nodes.GetIncidenceMatrix(&edges)

	// Imprimir la matriz de incidencia
	//for node, row := range incidenceMatrix {
	//	fmt.Printf("%s: %v\n", node, row)
	//}

	var html Types.Html

	html.GenerateTableIncidenceMatrix(&edges, incidenceMatrix)

	// Generar la lista de vértices
	vertexList := nodes.GenerateVertexList()
	//fmt.Println(vertexList)

	// Generar la función de incidencia
	incidenceFunction := edges.IncidenceFunc()
	//fmt.Println(incidenceFunction)

	// Calcular la cantidad de grados de cada vértice

	degreeCounts := incidenceMatrix.CalculateDegreeCounts()
	//fmt.Println(degreeCounts)

	graph := Algorith.BuildGraph(edges)
	subgraph := make([]Types.Graph, 2)
	subGraphType := reflect.TypeOf(r.SubGraph).Kind()
	//fmt.Println(subGraphType)
	switch subGraphType {
	case reflect.Array:
		var n Types.Nodes
		for _, node := range r.SubGraph.([]string) {
			n = append(n, node)
		}
		for i := range subgraph {
			subgraph[i] = Algorith.ExtractSubgraph(graph, n)
		}

	case reflect.Float64:
		for i := range subgraph {
			subgraph[i] = Algorith.ExtractRandomSubgraph(graph, int(r.SubGraph.(float64)))
		}
	}

	return GraphResponse{
		Graph:             graph,
		IncidenceMatrix:   incidenceMatrix,
		IncidenceFunction: incidenceFunction,
		VertexList:        vertexList,
		DegreeCounts:      degreeCounts,
		Html:              html,
		SimplePath:        simplePath,
		SubGraph:          subgraph,
	}
}

type RequestGraph struct {
	Graph Types.Graph `json:"graph"`
}

type RequestLength struct {
	RequestGraph
	Start string `json:"start"`
	End   string `json:"end"`
}

func main() {
	//logrus.Infof("beginning application")
	//err := godotenv.Load()
	//if err != nil {
	//	log.Fatalf("err loading: %v", err)
	//}
	//// Inicializa la base de datos
	//global.Load(c.InitDB())
	app := fiber.New()
	app.Get("/", func(c *fiber.Ctx) error {
		return c.SendString("Hello, World!")
	})

	app.Get("/graph", func(c *fiber.Ctx) error {
		var r Request
		err := c.BodyParser(&r)
		if err != nil {
			return err
		}
		if r == (Request{}) {
			return c.SendString("No se recibió ningún dato")
		}
		logrus.Infof("Request: %+v", r)
		return c.JSON(startA(r))
	})

	app.Post("/distance-matrix", func(c *fiber.Ctx) error {
		var request RequestGraph
		err := c.BodyParser(&request)
		if err != nil {
			return err
		}
		if len(request.Graph.Nodes) == 0 {
			// El campo request está vacío
			return c.Status(fiber.StatusBadRequest).JSON(fiber.Map{
				"message": "El campo 'request' no puede estar vacío",
			})
		}

		matrix := Algorith.DistanceMatrix(request.Graph)
		tableHTML := Algorith.FormatMatrixAsHTMLTable(matrix, request.Graph.Nodes, false)

		return c.SendString(tableHTML)
	})

	app.Post("/length", func(c *fiber.Ctx) error {
		var request RequestLength
		err := c.BodyParser(&request)
		if err != nil {
			return err
		}
		if len(request.Graph.Nodes) == 0 {
			// El campo request está vacío
			return c.Status(fiber.StatusBadRequest).JSON(fiber.Map{
				"message": "El campo 'request' no puede estar vacío",
			})
		}

		excentricity, excentricityMatrix := Algorith.FindAllExcentricity(request.Graph, request.Start)
		htmlTable := Algorith.FormatMatrixAsHTMLTable(excentricityMatrix, request.Graph.Nodes, true)
		length := Algorith.GetAllPaths(request.Graph, request.Start, request.End)
		//logrus.Println(length, excentricity, table)
		return c.JSON(fiber.Map{
			"length": length,
			"excentricity": fiber.Map{
				"list":  excentricity,
				"table": htmlTable,
			},
		})
		//return c.SendString(table)
	})

	app.Post("/center-radio-diametro", func(c *fiber.Ctx) error {
		var request RequestLength
		err := c.BodyParser(&request)
		if err != nil {
			return err
		}
		if len(request.Graph.Nodes) == 0 {
			// El campo request está vacío
			return c.Status(fiber.StatusBadRequest).JSON(fiber.Map{
				"message": "El campo 'request' no puede estar vacío",
			})
		}
		center, min, list := Algorith.FindGraphCenter(request.Graph)
		radio := Algorith.FindGraphRadius(request.Graph, min)
		diametro := Algorith.CalculateDiameter(request.Graph)
		return c.JSON(fiber.Map{
			"center":   center,
			"radio":    radio,
			"diametro": diametro,
			"list":     list,
		})
	})

	app.Post("/dijkstra-matrix", func(c *fiber.Ctx) error {
		var request RequestLength
		err := c.BodyParser(&request)
		if err != nil {
			return err
		}
		if len(request.Graph.Nodes) == 0 {
			// El campo request está vacío
			return c.Status(fiber.StatusBadRequest).JSON(fiber.Map{
				"message": "El campo 'request' no puede estar vacío",
			})
		}

		//distances, paths := Dijkstra(request.Graph, request.Start)
		//logrus.Println(distances, paths)
		//formatShortestPaths(distances, paths, request.Start)

		numNodes := len(request.Graph.Nodes)
		matrix := Algorith.InitializeMatrix(numNodes)
		matrix = Algorith.SetDiagonalZeros(matrix)
		index := Algorith.GetIndex(request.Start, request.Graph.Nodes)

		d, p := Algorith.FillMatrixWithEdges(matrix, request.Graph.Edges, request.Graph.Nodes, index)

		// Calcular el número medio de enlaces
		size := len(d)
		averageLinks := Algorith.CalculateAverageLinks(d, size)
		logrus.Println(averageLinks)
		return c.JSON(fiber.Map{
			"matrix":        Algorith.FormatShortestPaths(d, p, index, request.Graph.Nodes),
			"average_links": fmt.Sprintf("El número medio de enlaces que un paquete tiene que atravesar (en promedio) es: %v", averageLinks),
		})
		//return c.SendString(formatShortestPaths(d, p, index, request.Graph.Nodes))
	})

	//routes.SetupRoutes(app)
	//app.Use(middlewares.RouteLogger(app))

	logrus.Fatal(app.Listen(":3000"))
}

//func Dijkstra(graph Graph, source string) ([][]string, [][]float64) {
//	// Inicializar las distancias y rutas más cortas
//	size := len(graph.Nodes)
//	distances := make([][]float64, size)
//	resultMatrix := make([][]string, size)
//
//	for i := 0; i < size; i++ {
//		distances[i] = make([]float64, size)
//		resultMatrix[i] = make([]string, size)
//		for j := 0; j < size; j++ {
//			distances[i][j] = math.Inf(1)
//			resultMatrix[i][j] = "-"
//		}
//	}
//
//	// Asignar las distancias de los enlaces conocidos
//	for _, edge := range graph.Edges {
//		node1Index := getNodeIndex(graph.Nodes, edge.Node1)
//		node2Index := getNodeIndex(graph.Nodes, edge.Node2)
//		weight := parseWeight(edge.Weighted)
//
//		distances[node1Index][node2Index] = weight
//		distances[node2Index][node1Index] = weight
//		resultMatrix[node1Index][node2Index] = edge.Id
//		resultMatrix[node2Index][node1Index] = edge.Id
//	}
//
//	// Inicializar la distancia al nodo fuente como 0
//	sourceIndex := getNodeIndex(graph.Nodes, source)
//	distances[sourceIndex][sourceIndex] = 0
//
//	// Conjunto de nodos visitados
//	visited := make([]bool, size)
//
//	// Iterar hasta visitar todos los nodos
//	for count := 0; count < size; count++ {
//		// Encontrar el nodo con la distancia mínima
//		u := MinDistance(distances, visited)
//
//		// Marcar el nodo como visitado
//		visited[u] = true
//
//		// Actualizar las distancias de los nodos adyacentes
//		for v := 0; v < size; v++ {
//			if !visited[v] && distances[u][v] != math.Inf(1) && distances[u][v]+distances[u][u] < distances[u][v] {
//				distances[u][v] = distances[u][v] + distances[u][u]
//				resultMatrix[u][v] = resultMatrix[u][u] + "->" + graph.Nodes[v]
//			}
//		}
//	}
//
//	return resultMatrix, distances
//}

//func findShortestPaths(graph Graph, vertex string) string {
//	// Obtener el índice del vértice en el grafo
//	numNodes := len(graph.Nodes)
//	matrix := initializeMatrix(numNodes)
//	setDiagonalZeros(matrix)
//	//formattedPaths := fillMatrixWithEdges(matrix, graph.Edges, graph.Nodes)
//	//return formattedPaths
//}
