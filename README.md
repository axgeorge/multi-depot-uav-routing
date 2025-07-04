# Multi-Depot UAV Routing

<p align="center">
<img width="640" height="480" alt="Image" src="https://github.com/user-attachments/assets/20d67976-c63c-4e6d-8abb-f961f78118dd" />
</p>

This project generates and solves instances of a **Multi-Depot UAV Routing Problem**. It supports:

- Multiple depots (UAVs)
- Random instance generation
- Solution plotting

## Getting Started

### 1. Clone the Repository

```
git clone https://github.com/axgeorge/multi-depot-uav-routing.git
cd multi-depot-uav-routing
```
### 2. Create a Virtual Environment

```
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```
pip install -r requirements.txt
```

## LKH Setup

This project uses LKH (Lin-Kernighan-Helsgaun) to solve underlying TSP structures.

### 1. Download and Compile LKH

Download from: http://webhotel4.ruc.dk/~keld/research/LKH/ and follow instructions.

### 2. Set the LKH Path

In your `main.py`, update the `LKH_path` variable to point to the directory containing the compiled LKH binary:

```python
LKH_path = "/absolute/path/to/LKH-2.0.9/"
```

## Running the Project

Edit `main.py` to configure your instance generation parameters:

```python
num_vehicles = 10
num_targets = 100
vmin, vmax = 1, 1
xmin, xmax, ymin, ymax = -50, 50, -50, 50
```

Then, run the program:

```
python main.py
```
