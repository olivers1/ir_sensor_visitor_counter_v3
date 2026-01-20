<!DOCTYPE html>
<html>
<head>
<style>
    td, th {
        border: 1px solid black;
        padding: 10px;
        text-align: center;
    }
</style>
</head>

<body>

<table>
    <thead>
        <tr>
            <th>Timestamp (ms)</th>
            <th>Date & Time</th>
            <th>Movement Direction</th>
        </tr>
    </thead>
    <tbody id="table-body">
        <!-- Data will be inserted here -->
    </tbody>
</table>

<script>
async function loadData() {
    try {
        const response = await fetch('fetch_observations.php');
        const data = await response.json();

        const tbody = document.getElementById('table-body');
        tbody.innerHTML = '';

        data.forEach(row => {
            const timestamp = Number(row.timestamp); // ensure it’s a number

            let formattedDate = '';
            if (!isNaN(timestamp)) {
                const date = new Date(timestamp); // timestamp is in milliseconds
                formattedDate = date.toLocaleString(); // human-readable
            } else {
                console.warn('Invalid timestamp:', row.timestamp);
            }

            const tr = document.createElement('tr');
            tr.innerHTML = `
                <td>${row.timestamp}</td> <!-- original timestamp -->
                <td>${formattedDate}</td> <!-- converted date-time -->
                <td>${row.movement_direction}</td>
            `;

            tbody.appendChild(tr);
        });
    } catch (err) {
        console.error('Failed to load data', err);
    }
}

// Load immediately
loadData();

// Refresh every 5 seconds
setInterval(loadData, 5000);
</script>

</body>
</html>
