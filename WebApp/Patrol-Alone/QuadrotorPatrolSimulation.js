// Get canvas and context
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

// Set canvas size
canvas.width = 600;
canvas.height = 500;

// Quadrotor UAV position and size
let x = canvas.width / 2;
let y = canvas.height / 2;
const size = 20;

// Target area dimensions
const targetAreaWidth = 6 * 100;
const targetAreaHeight = 5 * 100;

// Draw target area
ctx.rect(0, 0, targetAreaWidth, targetAreaHeight);
ctx.stroke();

// Set initial waypoint
let targetX = Math.random() * (targetAreaWidth - size);
let targetY = Math.random() * (targetAreaHeight - size);

// Travel speed
const speed = 16;

// Array to store traversed locations
const visitedLocations = [];

// Array to store unvisited locations
let unvisitedLocations = [];
for (let i = 0; i < targetAreaWidth; i += size) {
  for (let j = 0; j < targetAreaHeight; j += size) {
    unvisitedLocations.push({ x: i, y: j });
  }
}
totalLocations = unvisitedLocations.length


// Draw grid
const drawGrid = () => {
  ctx.strokeStyle = "black";
  for (let i = size; i < targetAreaWidth; i += size) {
    ctx.beginPath();
    ctx.moveTo(i, 0);
    ctx.lineTo(i, targetAreaHeight);
    ctx.stroke();
  }
  for (let i = size; i < targetAreaHeight; i += size) {
    ctx.beginPath();
    ctx.moveTo(0, i);
    ctx.lineTo(targetAreaWidth, i);
    ctx.stroke();
  }
};

const drawCanvas = () => {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.rect(0, 0, targetAreaWidth, targetAreaHeight);
  ctx.stroke();
  drawGrid();
  ctx.fillStyle = "gray";
  visitedLocations.forEach(location => {
    ctx.fillRect(location.x, location.y, size, size);
  });
  ctx.fillStyle = "red";
  ctx.fillRect(x, y, size, size);
  ctx.fillStyle = "blue";
  ctx.fillRect(targetX, targetY, size, size);
};

const updateLocations = (currentLocation) => {
  const copterWidth = size * 2;
  const copterHeight = size * 2;

  const copterX = currentLocation.x - (copterWidth - size) / 2;
  const copterY = currentLocation.y - (copterHeight - size) / 2;

  for (let i = unvisitedLocations.length - 1; i >= 0; i--) {
    const location = unvisitedLocations[i];
    if (location.x >= copterX && location.x <= copterX + copterWidth &&
        location.y >= copterY && location.y <= copterY + copterHeight) {
      visitedLocations.push(location);
      unvisitedLocations.splice(i, 1);
    }
  }
};

const calculateWeight = (location, currentLocation) => {
  let penalty = 1;
  if (visitedLocations.findIndex(loc => loc.x === location.x && loc.y === location.y) !== -1) {
    penalty = 2 + visitedLocations.length;
  }
  const unvisitedFactor = unvisitedLocations.length / totalLocations;
  return penalty + unvisitedFactor * -1 +
    Math.sqrt(Math.pow(location.x - currentLocation.x, 2) + Math.pow(location.y - currentLocation.y, 2));
};

const getDistances = (currentLocation) => {
  const distances = [];
  unvisitedLocations.forEach(location => {
    distances.push({
      x: location.x,
      y: location.y,
      weight: calculateWeight(location, currentLocation)
    });
  });
  return distances;
};

const sortDistances = distances => distances.sort((a, b) => a.weight - b.weight);

const setTargetWaypoint = (distances, currentLocation) => {
  let adjacents = distances.filter(distance =>
    (Math.abs(distance.x - currentLocation.x) <= size && distance.y === currentLocation.y) ||
    (Math.abs(distance.y - currentLocation.y) <= size && distance.x === currentLocation.x)
  );
  
  if (adjacents.length > 0) {
    distances = adjacents;
  }

  const sortedDistances = sortDistances(distances);
  let randomIndex;
  if (unvisitedLocations.length / totalLocations <= 0.05) {
    randomIndex = 0;
  } else {
    randomIndex = Math.floor(Math.random() * sortedDistances.length);
  }
  const selectedWaypoint = sortedDistances[randomIndex];
  targetX = selectedWaypoint.x;
  targetY = selectedWaypoint.y;
  unvisitedLocations = unvisitedLocations.filter(
    loc => loc.x !== targetX || loc.y !== targetY
  );
  visitedLocations.push({ x: targetX, y: targetY });
};

const move = () => {
  let angle = Math.atan2(targetY - y, targetX - x) + (Math.random() - 0.5) * 0.1;
  x = x + speed * Math.cos(angle);
  y = y + speed * Math.sin(angle);

  drawCanvas();

  const currentLocation = { x: Math.floor(x / size) * size, y: Math.floor(y / size) * size };

  updateLocations(currentLocation);

  if (Math.abs(x - targetX) < speed && Math.abs(y - targetY) < speed) {
    if (unvisitedLocations.length > 0) {
      const distances = getDistances(currentLocation);
      setTargetWaypoint(distances, currentLocation);
    }
  }
  printUnvisited();
  requestAnimationFrame(move);
};






  //##############################################

  let startTime = 0;
  let timer = null;
  
  const startTimer = () => {
    startTime = new Date().getTime();
    timer = setInterval(() => {
      const elapsedTime = new Date().getTime() - startTime;
      document.getElementById("timer").innerHTML = `${Math.floor(elapsedTime / 1000)}s`;
    }, 1000);
  };
  
  const stopTimer = () => {
    clearInterval(timer);
  };
  
  startTimer();

  const printUnvisited = () => document.getElementById("scores").innerHTML = `Unvisited: ${unvisitedLocations.length}`;

  //##############################################

  requestAnimationFrame(move);


  