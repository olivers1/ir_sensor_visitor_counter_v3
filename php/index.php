<!DOCTYPE html>
<html>
<head>
<style>
    td, th {
        border: 0.5px solid black;
        padding: 6px;
        text-align: center;
        font-family: helvetica;
        font-size: 14px;
    }

    /* Container for ASCII cat + title */
    .header-container {
        display: flex;
        align-items: center; /* vertically center title with cat */
        justify-content: left; /* align left */
    }

    /* ASCII cat styling */
    .ascii-cat {
        font-family: monospace;
        margin-right: 20px; /* space between cat and title */
        line-height: 1; /* tight spacing for ASCII art */
    }

    /* Title styling */
    h1 {
        font-family: helvetica;
        margin: 0; /* remove default margin */
    }

    /* Date/time display styling */
    #current-datetime {
        font-family: helvetica;
        font-size: 14px;
        margin-left: 5px;
        margin-bottom: 2px;
    }
</style>
</head>

<body>

<div class="header-container">
    <pre class="ascii-cat">
 /\_/\  
( o.o ) 
 > ^ <
    </pre>
    <h1>Cat ObServer</h1>
</div>

<!-- Current date/time display just above the table -->
<div id="current-datetime"></div>

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
            const timestamp = Number(row.timestamp);

            let formattedDate = '';
            if (!isNaN(timestamp)) {
                const date = new Date(timestamp);
                formattedDate = date.toLocaleString();
            } else {
                console.warn('Invalid timestamp:', row.timestamp);
            }

            const tr = document.createElement('tr');
            tr.innerHTML = `
                <td>${row.timestamp}</td>
                <td>${formattedDate}</td>
                <td>${row.movement_direction}</td>
            `;

            tbody.appendChild(tr);
        });
    } catch (err) {
        console.error('Failed to load data', err);
    }
}

// Update current date and time every second
function updateDateTime() {
    const now = new Date();
    document.getElementById('current-datetime').textContent = now.toLocaleString();
}

updateDateTime();
setInterval(updateDateTime, 1000);

loadData();
setInterval(loadData, 5000);
</script>

</body>
</html>
