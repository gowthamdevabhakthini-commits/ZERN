import pandas as pd
import matplotlib.pyplot as plt

# Replace this with the actual name of your CSV file
file_name = 'eog_test_sub_1_omkar_20260311_232732.csv' 

try:
    # 1. Load the data, ignoring the header lines that start with '#'
    df = pd.read_csv(file_name, comment='#')
    
    print("✅ Successfully loaded the data!")
    print("\n--- First 3 rows ---")
    print(df.head(3))
    
    # 2. Let's plot the RAW data to see what the eye movements look like!
    plt.figure(figsize=(12, 5))
    
    # Plotting Raw Channel 0 (Blue) and Raw Channel 1 (Red) against Time (ms)
    plt.plot(df['elapsed_ms'], df['raw_ch0'], label='Raw CH0', color='blue', alpha=0.7)
    plt.plot(df['elapsed_ms'], df['raw_ch1'], label='Raw CH1', color='red', alpha=0.7)
    
    plt.xlabel("Time (milliseconds)")
    plt.ylabel("Amplitude")
    plt.title("Raw EOG Data - Subject: Omkar")
    plt.legend()
    plt.grid(True)
    
    # This will open a window displaying your graph
    plt.show()

except Exception as e:
    print(f"❌ Error: {e}")