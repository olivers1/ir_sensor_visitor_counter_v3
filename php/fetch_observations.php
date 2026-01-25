<?php
# login credentials hidden as global variables
$conn = new mysqli(getenv('DB_HOST'), getenv('DB_USER'), getenv('DB_PASS'), getenv('DB_NAME'));

if ($conn->connect_errno) {
    http_response_code(500);
    exit("DB connection failed");
}

# fetch rows containing 'ENTRY' / 'EXIT'
$sql = "
    SELECT timestamp, movement_direction 
    FROM observations 
    
    ORDER BY timestamp DESC";
    #WHERE movement_direction = 'ENTRY' OR movement_direction = 'EXIT'
$result = $conn->query($sql);

$data = [];
if ($result && $result->num_rows > 0) {
    $data = $result->fetch_all(MYSQLI_ASSOC);
}

# count number of 'EXIT' and store it as number of excursions
$countSql = "
    SELECT COUNT(*) AS excursions
    FROM observations
    WHERE movement_direction = 'EXIT'
";
$countResult = $conn->query($countSql);

$excursions = 0;
if ($countResult) {
    $row = $countResult->fetch_assoc();
    $excursions = (int) $row['excursions'];
}

$statusNowSql = "
    SELECT timestamp, movement_direction
    FROM observations 
    WHERE movement_direction = 'ENTRY' OR movement_direction = 'EXIT'
    ORDER BY timestamp DESC 
    LIMIT 1
";
$statusNowResult = $conn->query($statusNowSql);

$statusLocation = 'Unknown'; // default value
$statusUpdated = null;
if ($statusNowResult && $statusNowResult->num_rows > 0) {
    $latestRow = $statusNowResult->fetch_assoc();

    // Determine status location
    $direction = strtoupper(trim($latestRow['movement_direction']));
    if ($direction === 'EXIT') {
        $statusLocation = 'Outside';
    } elseif ($direction === 'ENTRY') {
        $statusLocation = 'Inside';
    }

    $statusUpdated = (int) $latestRow['timestamp'];
}

$statusTodaySql = "
    SELECT timestamp, movement_direction
    FROM observations
    WHERE timestamp / 1000 >= UNIX_TIMESTAMP(CURDATE())
      AND timestamp / 1000 < UNIX_TIMESTAMP(CURDATE() + INTERVAL 1 DAY)
";
$statusTodayResult = $conn->query($statusTodaySql);

$statusToday = [];
$excursionsToday = 0;
$timeOutsideToday = 0;
$statusToday = [];
$excursionsToday = 0;
$timeOutsideToday = 0;

$lastExitTimestamp = null;
$nowMs = round(microtime(true) * 1000);

if ($statusTodayResult && $statusTodayResult->num_rows > 0) {
    $statusToday = $statusTodayResult->fetch_all(MYSQLI_ASSOC);

    foreach ($statusToday as $row) {
        $direction = strtoupper(trim($row['movement_direction']));
        $timestamp = (int) $row['timestamp'];

        if ($direction === 'EXIT') {
            $excursionsToday++;
            $lastExitTimestamp = $timestamp;
        }

        if ($direction === 'ENTRY' && $lastExitTimestamp !== null) {
            $timeOutsideToday += ($timestamp - $lastExitTimestamp);
            $lastExitTimestamp = null;
        }
    }

    // Still outside at end of day → count until now
    if ($lastExitTimestamp !== null) {
        $timeOutsideToday += ($nowMs - $lastExitTimestamp);
    }
}

$timeOutsideToday = msToHMS($timeOutsideToday);

function msToHMS($ms) {
    $seconds = floor($ms / 1000);
    $hours = floor($seconds / 3600);
    $minutes = floor(($seconds % 3600) / 60);
    $seconds = $seconds % 60;

    return sprintf("%02d:%02d:%02d", $hours, $minutes, $seconds);
}

$conn->close();

header('Content-Type: application/json');
echo json_encode([
    'excursions' => $excursions,
    'observations' => $data,
    'statusLocation' => $statusLocation,
    'statusUpdated' => $statusUpdated,
    'excursionsToday' => $excursionsToday,
    'timeOutsideToday' => $timeOutsideToday
]);
