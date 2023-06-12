package Algorith

import (
	"fmt"
	"github.com/sirupsen/logrus"
	"math"
	"math/rand"
	"regexp"
	"sinectis-graphs/Types"
	"strconv"
	"strings"
	"time"
)

// Función de búsqueda de camino simple
func FindSimplePath(start string, end string, edges Types.Edges) []string {
	graph := make(Types.GraphMap)
	for _, edge := range edges {
		graph[edge.Node1] = append(graph[edge.Node1], edge.Node2)
		graph[edge.Node2] = append(graph[edge.Node2], edge.Node1)
	}

	visited := make(map[string]bool)
	for vertex := range graph {
		visited[vertex] = false
	}

	var path []string

	if Dfs(graph, start, end, visited, &path) {
		result := make([]string, len(path)-1)

		for i := 0; i < len(path)-1; i++ {
			result[i] = fmt.Sprintf("(%s, %s)", path[i], path[i+1])
		}

		return result
	}

	return nil
}

func GetSimplePath(start string, end string, edges Types.Edges) string {
	path := FindSimplePath(start, end, edges)
	var SimplePath string
	//
	if path != nil {
		// Imprimir el resultado
		SimplePath += fmt.Sprintf("Camino simple\n")
		SimplePath += fmt.Sprintf("C = {" + strings.Join(path, ", ") + "}")
		// Aquí puedes agregar la longitud total del camino si proporcionas información sobre las distancias
	} else {
		SimplePath = fmt.Sprintf("No se encontró un camino simple entre las ciudades especificadas.")
	}
	return SimplePath
}

func FilterInput(e string) (Types.Nodes, Types.Edges) {
	//re := regexp.MustCompile(`/(\w+) *= *\(([^,]+), *([^)]+)\)/`)
	e = strings.ReplaceAll(e, "\n", "")
	re2 := regexp.MustCompile(`(\w+)\s*=\s*\(([^,]+),([^)]+)\)`)
	re := regexp.MustCompile(`(\w+)\s*=\s*\(([^,]+),([^)]+)\)\s*\[([^]]+)\]`)
	matches := re.FindAllStringSubmatch(e, -1)
	if len(matches) == 0 {
		matches = re2.FindAllStringSubmatch(e, -1)
	}
	// Crear un slice de aristas y un slice de nodos
	var edges Types.Edges
	nodes := make(Types.Nodes, 0)
	logrus.Println(matches)
	for _, match := range matches {
		edge := Types.Edge{
			Id:    strings.TrimSpace(match[1]),
			Node1: strings.TrimSpace(match[2]),
			Node2: strings.TrimSpace(match[3]),
			//Weighted: strings.TrimSpace(match[4]),
		}
		edges = append(edges, edge)
		nodes = append(nodes, strings.TrimSpace(match[2]), strings.TrimSpace(match[3]))
	}
	return nodes, edges
}

var index = 0

func GetIndex(node string, nodes []string) int {
	for i, n := range nodes {
		if n == node {
			return i
		}
	}
	return -1
}

func CalculateAverageLinks(distances []float64, size int) float64 {
	totalDistance := 0.0
	totalPairs := 0

	for i := 0; i < size; i++ {
		for j := i + 1; j < size; j++ {
			if distances[i] != math.Inf(1) && distances[j] != math.Inf(1) {
				totalDistance += distances[j]
				totalPairs++
			}
		}
	}

	if totalPairs > 0 {
		averageLinks := totalDistance / float64(totalPairs)
		return averageLinks
	}

	return 0.0
}

func Dijkstra(matrix [][]float64, start int) ([]float64, map[int][]int) {
	size := len(matrix)
	visited := make([]bool, size)
	distances := make([]float64, size)
	paths := make(map[int][]int)

	// Inicializar todas las distancias con Infinito excepto el nodo de inicio
	for i := 0; i < size; i++ {
		distances[i] = math.Inf(1)
		paths[i] = []int{}
	}
	distances[start] = 0
	paths[start] = []int{start}

	// Encontrar el nodo con la distancia mínima en cada iteración
	for count := 0; count < size-1; count++ {
		u := MinDistance(distances, visited)
		visited[u] = true

		// Actualizar las distancias de los nodos adyacentes
		for v := 0; v < size; v++ {
			if !visited[v] && matrix[u][v] != math.Inf(1) && distances[u]+matrix[u][v] < distances[v] {
				distances[v] = distances[u] + matrix[u][v]
				//logrus.Infof("distances[%d] = %f", v, distances[v])
				paths[v] = append(paths[u], v)
				//logrus.Infof("paths[%d] = %v", v, paths[v])
			}
		}
	}
	// Asignar las distancias mínimas a la matriz de distancia
	for i := 0; i < size; i++ {
		matrix[start][i] = distances[i]
		matrix[i][start] = distances[i]
	}
	return distances, paths
}

func FormatShortestPaths(distance []float64, path map[int][]int, startNode int, nodes Types.Nodes) string {

	var formattedPaths strings.Builder

	var formartPath = func(path []int) string {
		var formattedPath strings.Builder
		for i, node := range path {
			if i == 0 {
				formattedPath.WriteString(nodes[node])
			} else {
				formattedPath.WriteString(fmt.Sprintf(" -> %s", nodes[node]))
			}
		}
		return formattedPath.String()
	}

	// Construir la tabla HTML
	formattedPaths.WriteString("<style>table, th, td {\n  border: 1px solid;\n} td, th {\npadding: 2px;\n}</style><table style='border-collapse: collapse;'>")
	formattedPaths.WriteString("<tr>")
	formattedPaths.WriteString("<th>Nodo Origen</th>")
	formattedPaths.WriteString("<th>Nodo Destino</th>")
	formattedPaths.WriteString("<th>Distancia Mínima</th>")
	formattedPaths.WriteString("<th>Recorrido</th>")
	formattedPaths.WriteString("</tr>")
	for i, node := range nodes {
		if i != startNode {
			formattedPaths.WriteString(fmt.Sprintf("<tr><td>%s</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n", nodes[startNode], node, distance[i], formartPath(path[i])))
		}
	}

	//formattedPaths.WriteString(fmt.Sprintf("<tr><td>%s</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n", nodes[i], nodes[j], distance, resultMatrix[i][j]))

	formattedPaths.WriteString("</table>")

	return formattedPaths.String()
}

func ParseWeight(weighted string) float64 {
	// Aquí puedes implementar la lógica para convertir el peso según tu necesidad
	// En este ejemplo, se asume que el peso es un número decimal
	weight, _ := strconv.ParseFloat(weighted, 64)
	return weight
}

func CalculateDiameter(graph Types.Graph) float64 {
	diameter := 0.0

	for i := 0; i < len(graph.Nodes); i++ {
		for j := i + 1; j < len(graph.Nodes); j++ {
			distance := CalculateShortestDistance(graph, graph.Nodes[i], graph.Nodes[j])
			if distance > diameter {
				diameter = distance
			}
		}
	}

	return diameter
}

func FindGraphRadius(graph Types.Graph, centerExcentricity float64) float64 {
	radius := math.Inf(1)
	for _, vertex := range graph.Nodes {
		excentricity := CalculateExcentricity(graph, vertex)
		if excentricity == centerExcentricity && excentricity < radius {
			radius = excentricity
		}
	}

	return radius
}

func FindGraphCenter(graph Types.Graph) (string, float64, []string) {
	minExcentricity := math.Inf(1)
	var centerVertex string
	excentricities := []string{}

	for _, vertex := range graph.Nodes {
		excentricity := CalculateExcentricity(graph, vertex)
		result := fmt.Sprintf("E(%s) = %.2f", vertex, excentricity)
		excentricities = append(excentricities, result)

		if excentricity < minExcentricity {
			minExcentricity = excentricity
			centerVertex = vertex
		}
	}
	centerResult := fmt.Sprintf("Vértice que posee la menor excentricidad E(%s) = %.2f", centerVertex, minExcentricity)
	excentricities = append(excentricities, centerResult)

	return centerVertex, minExcentricity, excentricities

}

func GetMaxExcentricity(row []float64) float64 {
	maxExcentricity := math.Inf(-1)

	for _, excentricity := range row {
		if excentricity > maxExcentricity {
			maxExcentricity = excentricity
		}
	}

	return maxExcentricity
}

func ParseFloatWeighted(edge Types.Edge) (float64, error) {
	weightedStr := strings.ReplaceAll(edge.Weighted, ",", ".")
	return strconv.ParseFloat(weightedStr, 64)
}

func FindAllExcentricity(graph Types.Graph, vertex string) ([]string, [][]float64) {
	var excentricities []string
	var maxExcentricity float64
	var excentricityMatrix [][]float64
	var maxVertex string

	for _, node := range graph.Nodes {
		excentricity := CalculateExcentricityForVertex(graph, vertex, node)
		if excentricity > maxExcentricity {
			maxExcentricity = excentricity
			maxVertex = vertex
		}
		result := FormatExcentricity(node, vertex, excentricity)
		excentricities = append(excentricities, result)
		excentricityRow := []float64{excentricity}
		excentricityMatrix = append(excentricityMatrix, excentricityRow)
	}

	result := fmt.Sprintf("La Distancia más alta que sería D (%s) cuyo valor es de %.2f el cual es el valor de excentricidad del vértice %s, E(%s) = %.2f.",
		maxVertex, maxExcentricity, maxVertex, maxVertex, maxExcentricity)
	excentricities = append(excentricities, result)

	return excentricities, excentricityMatrix
}

func FormatExcentricity(node string, vertex string, excentricity float64) string {
	result := fmt.Sprintf("E(%s,%s) = %.2f", vertex, node, excentricity)
	return result
}

func CalculateExcentricity(graph Types.Graph, vertex string) float64 {
	var maxDistance float64

	for _, targetNode := range graph.Nodes {
		if targetNode != vertex {
			distance := CalculateShortestDistance(graph, vertex, targetNode)
			if distance > maxDistance {
				maxDistance = distance
			}
		}
	}

	return maxDistance
}

func CalculateExcentricityForVertex(graph Types.Graph, vertex, node string) float64 {
	distance := CalculateShortestDistance(graph, vertex, node)
	return distance
}

func CalculateShortestDistance(graph Types.Graph, start, end string) float64 {
	distances := make(map[string]float64)
	for _, node := range graph.Nodes {
		distances[node] = math.Inf(1)
	}
	distances[start] = 0

	queue := []string{start}

	for len(queue) > 0 {
		current := queue[0]
		queue = queue[1:]

		if current == end {
			return distances[current]
		}

		for _, edge := range graph.Edges {
			if edge.Node1 == current {
				neighbor := edge.Node2
				weight, err := ParseFloatWeighted(edge)
				if err != nil {
					// Manejar el error si la conversión falla
					logrus.Fatal(err)
					// Puedes optar por omitir esta arista o asignar un valor predeterminado
					continue
				}
				alt := distances[current] + weight
				if alt < distances[neighbor] {
					distances[neighbor] = alt
					queue = append(queue, neighbor)
				}
			} else if edge.Node2 == current {
				neighbor := edge.Node1
				weight, err := ParseFloatWeighted(edge)
				if err != nil {
					// Manejar el error si la conversión falla
					logrus.Fatal(err)
					// Puedes optar por omitir esta arista o asignar un valor predeterminado
					continue
				}
				alt := distances[current] + weight
				if alt < distances[neighbor] {
					distances[neighbor] = alt
					queue = append(queue, neighbor)
				}
			}
		}
	}

	return math.Inf(1)
}

func GetAllPaths(graph Types.Graph, start, end string) []string {
	visited := make(map[string]bool)
	var currentPath []string
	var allPaths []string
	var minLength = math.Inf(1)

	FindAllPaths(graph, start, end, visited, currentPath, &allPaths, &minLength)
	index = 0

	for _, path := range allPaths {
		if length := GetPathLengthFromFormat(path); length < minLength {
			minLength = length
		}
	}

	minLengthString := fmt.Sprintf("D(%s, %s) = min (CS) = %.2f KM", start, end, minLength)
	allPaths = append(allPaths, minLengthString)

	return allPaths
}

func GetPathLengthFromFormat(path string) float64 {
	parts := strings.Split(path, " = ")
	if len(parts) < 2 {
		return math.Inf(1)
	}

	lengthStr := strings.TrimSpace(parts[len(parts)-1])
	length, err := strconv.ParseFloat(lengthStr, 64)
	if err != nil {
		return math.Inf(1)
	}

	return length
}

func FindAllPaths(graph Types.Graph, current, end string, visited map[string]bool, currentPath []string, allPaths *[]string, minLength *float64) {
	visited[current] = true
	currentPath = append(currentPath, current)

	if current == end {
		*allPaths = append(*allPaths, FormatPath(currentPath, graph))
		if length := CalculatePathLength(currentPath, graph); length < *minLength {
			*minLength = length
		}
	} else {
		for _, edge := range graph.Edges {
			if edge.Node1 == current {
				neighbor := edge.Node2
				if !visited[neighbor] {
					FindAllPaths(graph, neighbor, end, visited, currentPath, allPaths, minLength)
				}
			} else if edge.Node2 == current {
				neighbor := edge.Node1
				if !visited[neighbor] {
					FindAllPaths(graph, neighbor, end, visited, currentPath, allPaths, minLength)
				}
			}
		}
	}

	delete(visited, current)
	currentPath = currentPath[:len(currentPath)-1]

}

func FormatPath(path []string, graph Types.Graph) string {
	//formattedPath := strings.Join(path, " -> ")
	length := CalculatePathLength(path, graph)

	result := fmt.Sprintf("CS%d: ", index+1)
	for i := 0; i < len(path)-1; i++ {
		node1 := path[i]
		node2 := path[i+1]
		for _, edge := range graph.Edges {
			if (edge.Node1 == node1 && edge.Node2 == node2) || (edge.Node1 == node2 && edge.Node2 == node1) {
				result += fmt.Sprintf("%s - %s - ", node1, edge.Id)
				break
			}
		}
	}
	result += fmt.Sprintf("%s <-------> Longitud de CS%d = %f KM\n ", path[len(path)-1], index+1, length)
	//return fmt.Sprintf("%s: %f", formattedPath, length)
	index++
	return result
}

func CalculatePathLength(path []string, graph Types.Graph) float64 {
	length := 0.0
	for i := 0; i < len(path)-1; i++ {
		node1 := path[i]
		node2 := path[i+1]
		for _, edge := range graph.Edges {
			if (edge.Node1 == node1 && edge.Node2 == node2) || (edge.Node1 == node2 && edge.Node2 == node1) {
				weighted, err := strconv.ParseFloat(edge.Weighted, 64)
				if err == nil {
					length += weighted
				}
				break
			}
		}
	}
	return length
}

func DistanceMatrix(graph Types.Graph) [][]float64 {
	numNodes := len(graph.Nodes)
	matrix := InitializeMatrix(numNodes)
	matrix = SetDiagonalZeros(matrix)
	FillMatrixWithEdges(matrix, graph.Edges, graph.Nodes, -1)
	return matrix
}

func FormatMatrixAsHTMLTable(matrix [][]float64, nodes Types.Nodes, list bool) string {
	var sb strings.Builder

	sb.WriteString("<style>table, th, td {\n  border: 1px solid;\n} td, th {\npadding: 2px;\n}</style><table style='border-collapse: collapse;'>")
	// Encabezado de la tabla (nombres de los nodos)

	if list {
		sb.WriteString("<tr><th>Vértice</th>")
		sb.WriteString("<th>")
		sb.WriteString("Exec(Vi)")
		sb.WriteString("</th>")
	} else {
		sb.WriteString("<tr><th></th>")
		for _, node := range nodes {
			sb.WriteString("<th>")
			sb.WriteString(node)
			sb.WriteString("</th>")
		}
	}

	sb.WriteString("</tr>")
	for i, row := range matrix {
		sb.WriteString("<tr>")

		sb.WriteString("<th>")
		sb.WriteString(nodes[i])
		sb.WriteString("</th>")

		for j, value := range row {
			cell := fmt.Sprintf("%.2f", value)
			if i == j {
				sb.WriteString("<td style='background-color: yellow;'>")
			} else {
				sb.WriteString("<td>")
			}
			sb.WriteString(cell)
			sb.WriteString("</td>")
		}
		sb.WriteString("</tr>")
	}
	sb.WriteString("</table>")

	return sb.String()
}

func InitializeMatrix(size int) [][]float64 {
	matrix := make([][]float64, size)
	for i := 0; i < size; i++ {
		matrix[i] = make([]float64, size)
		for j := 0; j < size; j++ {
			matrix[i][j] = math.Inf(1)
		}
	}
	return matrix
}

func SetDiagonalZeros(matrix [][]float64) [][]float64 {
	for i := 0; i < len(matrix); i++ {
		matrix[i][i] = 0
	}
	return matrix
}

func InitMatrixInf(matrix [][]float64) [][]float64 {
	for i := 0; i < len(matrix); i++ {
		for j := 0; j < len(matrix); j++ {
			matrix[i][j] = math.Inf(1)
		}
	}
	return matrix
}

func FillMatrixWithEdges(matrix [][]float64, edges Types.Edges, nodes Types.Nodes, startNode int) ([]float64, map[int][]int) {
	nodeIndex := make(map[string]int)
	for i, node := range nodes {
		nodeIndex[node] = i
	}

	// Inicializar todas las distancias con Infinito
	matrix = InitMatrixInf(matrix)

	// Asignar las distancias entre nodos vecinos
	for _, edge := range edges {
		node1 := nodeIndex[edge.Node1]
		node2 := nodeIndex[edge.Node2]
		weighted, err := ParseFloatWeighted(edge)
		if err != nil {
			// Manejar el error si la conversión falla
			logrus.Fatal(err)
			// Puedes optar por omitir esta arista o asignar un valor predeterminado
			continue
		}
		matrix[node1][node2] = weighted
		matrix[node2][node1] = weighted
	}
	var d []float64
	var p map[int][]int
	if startNode == -1 {
		for i := 0; i < len(matrix); i++ {
			Dijkstra(matrix, i)
		}
	} else {
		d, p = Dijkstra(matrix, startNode)
	}
	return d, p
}

func MinDistance(distances []float64, visited []bool) int {
	min := math.Inf(1)
	minIndex := -1
	for i, distance := range distances {
		if !visited[i] && distance < min {
			min = distance
			minIndex = i
		}
	}
	return minIndex
}

// Función auxiliar para obtener el índice de un nodo en el slice de nodos
func GetNodeIndex(nodes Types.Nodes, node string) int {
	for i, n := range nodes {
		if n == node {
			return i
		}
	}
	return -1
}

func FusionVertices(graph Types.Graph, vertexA string, vertexB string) Types.Graph {
	// Crear un nuevo grafo para almacenar el resultado de la fusión
	mergedGraph := Types.Graph{
		Nodes: make(Types.Nodes, 0),
		Edges: make(Types.Edges, 0),
	}

	// Copiar los nodos del grafo original excepto los vértices fusionados
	for _, node := range graph.Nodes {
		if node != vertexA && node != vertexB {
			mergedGraph.Nodes = append(mergedGraph.Nodes, node)
		}
	}

	// Copiar las aristas del grafo original excepto las aristas incidentes en los vértices fusionados
	for _, edge := range graph.Edges {
		if edge.Node1 != vertexA && edge.Node1 != vertexB && edge.Node2 != vertexA && edge.Node2 != vertexB {
			mergedGraph.Edges = append(mergedGraph.Edges, edge)
		}
	}

	// Crear una nueva arista que conecte los vértices fusionados en el nuevo vértice
	mergedEdge := Types.Edge{
		Id:    "merged",
		Node1: vertexA,
		Node2: vertexB,
	}
	mergedGraph.Edges = append(mergedGraph.Edges, mergedEdge)

	// Agregar el nuevo vértice al grafo fusionado
	mergedGraph.Nodes = append(mergedGraph.Nodes, "merged")

	return mergedGraph
}

func BuildGraph(edges Types.Edges) Types.Graph {
	graph := Types.Graph{
		Nodes: Types.Nodes{},
		Edges: edges,
	}

	nodeSet := make(map[string]bool)
	for _, edge := range edges {
		nodeSet[edge.Node1] = true
		nodeSet[edge.Node2] = true
	}

	for node := range nodeSet {
		graph.Nodes = append(graph.Nodes, node)
	}

	return graph
}

func ExtractSubgraph(graph Types.Graph, subGraphNodes []string) Types.Graph {
	subGraph := Types.Graph{
		Nodes: make(Types.Nodes, 0),
		Edges: make([]Types.Edge, 0),
	}

	nodeSet := make(map[string]bool)
	for _, node := range subGraphNodes {
		nodeSet[node] = true
	}

	for _, edge := range graph.Edges {
		if nodeSet[edge.Node1] && nodeSet[edge.Node2] {
			subGraph.Edges = append(subGraph.Edges, edge)
			nodeSet[edge.Node1] = true
			nodeSet[edge.Node2] = true
		}
	}

	for node := range nodeSet {
		subGraph.Nodes = append(subGraph.Nodes, node)
	}

	return subGraph
}

func ExtractRandomSubgraph(graph Types.Graph, numNodes int) Types.Graph {
	subgraphNodes := GetRandomNodes(graph, numNodes)
	fmt.Printf("nodes %v \n", subgraphNodes)
	return ExtractSubgraph(graph, subgraphNodes)
}

func GetRandomNodes(graph Types.Graph, numNodes int) Types.Nodes {
	rand.Seed(time.Now().UnixNano())

	// Obtener todos los nodos disponibles en el grafo
	availableNodes := make(Types.Nodes, len(graph.Nodes))
	copy(availableNodes, graph.Nodes)

	// Verificar si el número de nodos solicitados es mayor que el número total de nodos disponibles
	if numNodes > len(availableNodes) {
		numNodes = len(availableNodes)
	}

	// Elegir nodos aleatorios
	randomNodes := make(Types.Nodes, 0, numNodes)
	for i := 0; i < numNodes; i++ {
		randomIndex := rand.Intn(len(availableNodes))
		randomNode := availableNodes[randomIndex]
		randomNodes = append(randomNodes, randomNode)

		// Eliminar el nodo aleatorio del conjunto de nodos disponibles
		availableNodes = append(availableNodes[:randomIndex], availableNodes[randomIndex+1:]...)
	}

	return randomNodes
}

func Contains(list Types.Nodes, element string) bool {
	for _, item := range list {
		if item == element {
			return true
		}
	}
	return false
}

// Calcular la cantidad de grados de cada vértice
func CalculateDegree(graph Types.GraphMap) map[string]int {
	degree := make(map[string]int)

	for node, neighbors := range graph {
		degree[node] = len(neighbors)
	}

	return degree
}

// Función de búsqueda en profundidad (DFS)
func Dfs(graph Types.GraphMap, start string, end string, visited map[string]bool, path *[]string) bool {
	visited[start] = true
	*path = append(*path, start)

	if start == end {
		return true
	}

	for _, neighbor := range graph[start] {
		if !visited[neighbor] {
			if Dfs(graph, neighbor, end, visited, path) {
				return true
			}
		}
	}

	*path = (*path)[:len(*path)-1]
	return false
}
