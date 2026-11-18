import json
import yaml
import csv
from datetime import datetime
import os

def save_path_to_file(path, filename=None):
    """
    Salva o path gerado pelo move_arm em um arquivo
    
    Args:
        path: lista de strings com os ângulos das juntas
        filename: nome do arquivo (opcional)
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robot_path_{timestamp}.txt"
    
    with open(filename, 'w') as f:
        f.write(f"# Path generated at: {datetime.now()}\n")
        f.write(f"# Number of waypoints: {len(path)}\n")
        f.write("# Format: joint1 joint2 joint3 joint4 joint5 joint6 joint7 (in degrees)\n")
        f.write("#" + "="*60 + "\n\n")
        
        for i, waypoint in enumerate(path):
            f.write(f"Waypoint {i+1:03d}: {waypoint}\n")
    
    print(f"[INFO] Path saved to: {filename}")
    return filename

def save_path_to_json(path, filename=None):
    """
    Salva o path em formato JSON
    
    Args:
        path: lista de strings com os ângulos das juntas
        filename: nome do arquivo (opcional)
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robot_path_{timestamp}.json"
    
    # Converter cada waypoint de string para lista de floats
    waypoints = []
    for waypoint_str in path:
        # Converte "45.0 60.0 0.0 -25.0 0.0 95.0 0.0" para [45.0, 60.0, 0.0, -25.0, 0.0, 95.0, 0.0]
        angles = [float(x) for x in waypoint_str.split()]
        waypoints.append(angles)
    
    data = {
        "metadata": {
            "timestamp": datetime.now().isoformat(),
            "num_waypoints": len(path),
            "description": "Joint angles in degrees for KUKA iiwa"
        },
        "waypoints": waypoints
    }
    
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"[INFO] Path saved to JSON: {filename}")
    return filename

def save_path_to_csv(path, filename=None):
    """
    Salva o path em formato CSV
    
    Args:
        path: lista de strings com os ângulos das juntas
        filename: nome do arquivo (opcional)
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robot_path_{timestamp}.csv"
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Escrever cabeçalho
        writer.writerow(['waypoint_id', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7'])
        
        # Escrever dados
        for i, waypoint_str in enumerate(path):
            angles = [float(x) for x in waypoint_str.split()]
            writer.writerow([i+1] + angles)
    
    print(f"[INFO] Path saved to CSV: {filename}")
    return filename

def save_path_to_yaml(path, filename=None):
    """
    Salva o path em formato YAML
    
    Args:
        path: lista de strings com os ângulos das juntas
        filename: nome do arquivo (opcional)
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"robot_path_{timestamp}.yaml"
    
    # Converter para formato mais legível
    waypoints = []
    for waypoint_str in path:
        angles = [float(x) for x in waypoint_str.split()]
        waypoints.append(angles)
    
    data = {
        'metadata': {
            'timestamp': datetime.now().isoformat(),
            'num_waypoints': len(path),
            'robot': 'KUKA iiwa',
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        },
        'waypoints': waypoints
    }
    
    with open(filename, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)
    
    print(f"[INFO] Path saved to YAML: {filename}")
    return filename

def save_path_all_formats(path, base_filename=None):
    """
    Salva o path em todos os formatos (txt, json, csv, yaml)
    
    Args:
        path: lista de strings com os ângulos das juntas
        base_filename: nome base do arquivo (sem extensão)
    """
    if base_filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_filename = f"robot_path_{timestamp}"
    
    # Criar diretório para os arquivos
    output_dir = "robot_paths"
    os.makedirs(output_dir, exist_ok=True)
    
    # Salvar em cada formato
    save_path_to_file(path, os.path.join(output_dir, f"{base_filename}.txt"))
    save_path_to_json(path, os.path.join(output_dir, f"{base_filename}.json"))
    save_path_to_csv(path, os.path.join(output_dir, f"{base_filename}.csv"))
    save_path_to_yaml(path, os.path.join(output_dir, f"{base_filename}.yaml"))
    
    print(f"[INFO] All path files saved in '{output_dir}/'")
