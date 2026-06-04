import os
import json
import glob

# Get the absolute path of the directory the script is sitting in
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Go up one level to the root, then into the profiles directory
PROFILES_DIR = os.path.join(SCRIPT_DIR, '..', 'profiles')
INDEX_FILE = os.path.join(PROFILES_DIR, 'index.json')

def generate_index():
    # ---> NEW: Create the folder automatically if it doesn't exist <---
    os.makedirs(PROFILES_DIR, exist_ok=True)
    
    profiles_list = []
    
    # Grab all .json files in the profiles directory
    search_pattern = os.path.join(PROFILES_DIR, '*.json')

    for filepath in glob.glob(search_pattern):
        filename = os.path.basename(filepath)
        
        # We don't want to index the index itself
        if filename == 'index.json':
            continue
            
        with open(filepath, 'r', encoding='utf-8') as f:
            try:
                data = json.load(f)
                
                # Dig into the unified JSON structure to find the car's name
                if 'cars' in data and len(data['cars']) > 0:
                    car_name = data['cars'][0].get('car_model')
                    
                    if car_name:
                        profiles_list.append({
                            "name": car_name,
                            "file": filename
                        })
                    else:
                        print(f"Warning: No 'car_model' found in {filename}")
            
            except json.JSONDecodeError:
                print(f"Error: {filename} is not valid JSON. Skipping.")
            except Exception as e:
                print(f"Error processing {filename}: {e}")

    # Sort the list alphabetically by car name so the WiCan dropdown is organized
    profiles_list = sorted(profiles_list, key=lambda x: x['name'].lower())
    
    # Write the compiled list into index.json
    index_data = {"profiles": profiles_list}
    
    with open(INDEX_FILE, 'w', encoding='utf-8') as f:
        json.dump(index_data, f, indent=2)
        
    print(f"✅ Successfully generated index.json with {len(profiles_list)} profiles.")

if __name__ == '__main__':
    generate_index()
