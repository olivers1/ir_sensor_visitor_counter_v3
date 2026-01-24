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

$currentStatusSql = "
    SELECT timestamp, movement_direction
    FROM observations 
    WHERE movement_direction = 'ENTRY' OR movement_direction = 'EXIT'
    ORDER BY timestamp DESC 
    LIMIT 1
";
$currentResult = $conn->query($currentStatusSql);

$statusLocation = 'Unknown'; // default value
$statusUpdated = 'Unknown';

if ($currentResult && $currentResult->num_rows > 0) {
    $latestRow = $currentResult->fetch_assoc();

    // Determine current location
    $direction = strtoupper(trim($latestRow['movement_direction']));
    if ($direction === 'EXIT') {
        $statusLocation = 'Outside';
    } elseif ($direction === 'ENTRY') {
        $statusLocation = 'Inside';
    }

    // Store latest timestamp
    $latestTimestamp = $latestRow['timestamp'];

    // Convert timestamp to date/time
    // Assuming your timestamp is in milliseconds
    if (is_numeric($latestTimestamp)) {
        $seconds = (int)($latestTimestamp / 1000); // convert ms to seconds
        $statusUpdated = date('Y-m-d H:i:s', $seconds); // human-readable date/time
    } else {
        $statusUpdated = $latestTimestamp; // fallback in case it's already a string
    }
}





$data = [];

if ($result && $result->num_rows > 0) {
    $data = $result->fetch_all(MYSQLI_ASSOC);
}

$conn->close();

header('Content-Type: application/json');
echo json_encode([
    'excursions' => $excursions,
    'observations' => $data,
    'statusLocation' => $statusLocation,
    'statusUpdated' => $statusUpdated
]);
