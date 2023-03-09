package main

import (
	"bufio"
	"fmt"
	"math"
	"math/rand"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"time"
)

type Point3D struct {
	X float64
	Y float64
	Z float64
}
type Plane3D struct {
	A float64
	B float64
	C float64
	D float64
}
type Plane3DwSupport struct {
	Plane3D
	SupportSize int
}

func displayPoints(points []Point3D) {
	for index, value := range points {
		fmt.Println(index+1, " ", value)
	}
}

func stringToFloat64(word string) float64 {
	for true {
		if number, err := strconv.ParseFloat(word, 64); err == nil {
			return number
		}
	}
	return 0
}

func (point *Point3D) toString() (line string) {
	line = fmt.Sprintf("%f\t%f\t%f\n", point.X, point.Y, point.Z)
	return
}

// reads an XYZ file and returns a slice of Point3D
func ReadXYZ(filename string) []Point3D {
	var points []Point3D

	file, err := os.Open(filename)
	if err != nil {
		return points
	}
	defer file.Close()

	sb := bufio.NewScanner(file)
	sb.Split(bufio.ScanLines)
	sb.Scan()

	for sb.Scan() {
		line := sb.Text()
		words := strings.Fields(line)
		var newPoint Point3D
		newPoint.X = stringToFloat64(words[0])
		newPoint.Y = stringToFloat64(words[1])
		newPoint.Z = stringToFloat64(words[2])

		points = append(points, newPoint)
	}

	return points
}

// saves a slice of Point3D into an XYZ file
func SaveXYZ(filename string, points []Point3D) {
	// fmt.Println("write:", len(points), "to file ", filename)
	file, err := os.Create(filename)
	if err != nil {
		return
	}
	defer file.Close()
	file.WriteString("x\ty\tz\n")
	for index, value := range points {
		text := fmt.Sprint(index, " ", value.toString())
		if _, err := file.WriteString(text); err != nil {
			fmt.Println("Error while writing to a file")
			return
		}
	}
}

// computes the distance between points p1 and p2
func (p1 *Point3D) GetDistance(p2 *Point3D) float64 {
	distance := math.Sqrt(math.Pow(p1.X-p2.X, 2) + math.Pow(p1.Y-p2.Y, 2) + math.Pow(p1.Z-p2.Z, 2))
	return distance
}

// computes the distance between points to a plane
func (plane *Plane3D) GetDistance(p *Point3D) float64 {
	distance := (math.Abs(plane.A*p.X + plane.B*p.Y + plane.C*p.Z + plane.D)) / (math.Sqrt(math.Pow(plane.A, 2) + math.Pow(plane.B, 2) + math.Pow(plane.C, 2)))
	return distance
}

// computes the plane defined by a set of 3 points
func GetPlane(points []Point3D) Plane3D {
	x1 := points[0].X
	y1 := points[0].Y
	z1 := points[0].Z
	x2 := points[1].X
	y2 := points[1].Y
	z2 := points[1].Z
	x3 := points[2].X
	y3 := points[2].Y
	z3 := points[2].Z
	a1 := x2 - x1
	b1 := y2 - y1
	c1 := z2 - z1
	a2 := x3 - x1
	b2 := y3 - y1
	c2 := z3 - z1
	a := b1*c2 - b2*c1
	b := a2*c1 - a1*c2
	c := a1*b2 - b1*a2
	d := (-a*x1 - b*y1 - c*z1)

	return Plane3D{a, b, c, d}
}

// computes the number of required RANSAC iterations
func GetNumberOfIterations(confidence float64, percentageOfPointsOnPlane float64) int {
	k := (math.Log(1 - confidence)) / (math.Log(1 - math.Pow(percentageOfPointsOnPlane, 3)))
	return int(math.Ceil(k))
}

// computes the support of a plane in a set of points
func GetSupport(plane Plane3D, points []Point3D, eps float64) Plane3DwSupport {
	support := Plane3DwSupport{plane, 0}
	for _, value := range points {
		if (&plane).GetDistance(&value) <= eps {
			(support.SupportSize)++
		}
	}
	return support
}

// extracts the points that supports the given plane
// and returns them as a slice of points
func GetSupportingPoints(plane Plane3D, points []Point3D, eps float64) []Point3D {
	support := make([]Point3D, 0)
	for _, value := range points {
		if (&plane).GetDistance(&value) <= eps {
			support = append(support, value)
		}
	}
	return support
}

// creates a new slice of points in which all points
// belonging to the plane have been removed
func RemovePlane(plane Plane3D, points []Point3D, eps float64) []Point3D {
	newPoints := make([]Point3D, 0)

	for _, value := range points {
		if (&plane).GetDistance(&value) > eps {
			newPoints = append(newPoints, value)
		}
	}
	return newPoints
}

// Generate a ramdom point
func randomPoint(cloud []Point3D) Point3D {
	size := len(cloud)
	index := rand.Intn(size)
	// fmt.Println("Picked index: ", index)
	return cloud[index]
}

// Generate triplet of points
func randomTriplet(cloud []Point3D) [3]Point3D {
	return [3]Point3D{randomPoint(cloud), randomPoint(cloud), randomPoint(cloud)}
}

// takeN
func takeN(iteration int, cloud []Point3D, wg *sync.WaitGroup) <-chan [3]Point3D {
	ch := make(chan [3]Point3D)
	wg.Add(1)
	go func(c chan<- [3]Point3D) {
		defer func() {
			// fmt.Println("takeN done")
			wg.Done()
		}()
		for i := 0; i < iteration; i++ {
			c <- randomTriplet(cloud)
		}
		// fmt.Println("all points send")
		close(c)
	}(ch)
	return ch
}

// Fan In
func fanIn(chOut <-chan Plane3DwSupport, convergeCh chan<- Plane3DwSupport, wg *sync.WaitGroup) {
	defer func() {
		wg.Done()
	}()
	for support := range chOut {
		convergeCh <- support
	}
}

// Dominant plane identifier
func dominantIdentifier(dominant *Plane3DwSupport, convergeCh <-chan Plane3DwSupport, wg2 *sync.WaitGroup) {
	defer func() {
		// fmt.Println("dominant found")
		wg2.Done()
	}()

	for value := range convergeCh {
		if value.SupportSize > dominant.SupportSize {
			*dominant = value
		}
	}
}

func resetDominant(support *Plane3DwSupport) {
	support.SupportSize = 0
}

func main() {
	start := time.Now()
	var wg sync.WaitGroup
	var wg2 sync.WaitGroup

	dominant := Plane3DwSupport{Plane3D{0, 0, 0, 0}, 0}
	threadNumber := 3000

	filename := os.Args[1]
	confidence := stringToFloat64(os.Args[2])
	percentage := stringToFloat64(os.Args[3])
	eps := stringToFloat64(os.Args[4])
	extension := filepath.Ext(filename)
	file := strings.TrimRight(filename, extension)

	points := ReadXYZ(filename)
	iteration := GetNumberOfIterations(confidence, percentage)
	fmt.Println("Iteration: ", iteration)

	for i := 1; i <= 3; i++ {

		supportChs := make([]chan Plane3DwSupport, threadNumber)
		convergeCh := make(chan Plane3DwSupport)

		ch := takeN(iteration, points, &wg)

		//generate thread for plane estimator and supporting points finder
		for i := 0; i < threadNumber; i++ {
			supportChs[i] = make(chan Plane3DwSupport)

			wg.Add(1)
			go func(c <-chan [3]Point3D, index int) {
				defer func() {
					close(supportChs[index])
					// fmt.Println(index, "thread dies")
					wg.Done()
				}()

				// fmt.Println(index, " thread created")
				for triplet := range c {
					plane := GetPlane(triplet[:])
					supportChs[index] <- GetSupport(plane, points, eps)
				}
			}(ch, i)

		}

		//Fan in all the supports found in each thread
		for i := 0; i < threadNumber; i++ {
			wg.Add(1)
			go fanIn(supportChs[i], convergeCh, &wg)
		}

		wg2.Add(1)
		go dominantIdentifier(&dominant, convergeCh, &wg2)

		//Wait for all thread channel successfully output to converge
		wg.Wait()
		close(convergeCh)

		//Wait for the all support in converge channel successfully calculated
		wg2.Wait()
		fmt.Println("dominant", i, " : ", dominant)

		dominantPoints := GetSupportingPoints(dominant.Plane3D, points, eps)
		file1 := fmt.Sprint(file, "_p", strconv.Itoa(i), extension)
		SaveXYZ(file1, dominantPoints)

		points = RemovePlane(dominant.Plane3D, points, eps)
		resetDominant(&dominant)
	}

	file0 := fmt.Sprint(file, "_p0", extension)
	SaveXYZ(file0, points)

	elapsed := time.Since(start)
	fmt.Println("Process time:", elapsed, " with ", threadNumber, " threads and ", iteration, " iteration")

}

// go run main.go PointCloud1.xyz 0.99 0.1 0.1
