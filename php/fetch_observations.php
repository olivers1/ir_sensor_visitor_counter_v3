<?php
# login credentials hidden as global variables
$conn = new mysqli(getenv('DB_HOST'), getenv('DB_USER'), getenv('DB_PASS'), getenv('DB_NAME'));

if ($conn->connect_errno) {
    http_response_code(500);
    exit("DB connection failed");
}

$sql = "SELECT timestamp, movement_direction FROM observations ORDER BY timestamp DESC";
$result = $conn->query($sql);

$data = [];

if ($result && $result->num_rows > 0) {
    $data = $result->fetch_all(MYSQLI_ASSOC);
}

$conn->close();

header('Content-Type: application/json');
echo json_encode($data);
