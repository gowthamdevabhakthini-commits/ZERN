import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks

# --- STEP 4 SETUP: Define your thresholds ---
BLINK_THRESHOLD = 50       
MOVEMENT_THRESHOLD = 50    

def bandpass_filter(data, lowcut, highcut, fs, order=4):
    nyquist = 0.5 * fs
    low = lowcut / nyquist
    high = highcut / nyquist
    b, a = butter(order, [low, high], btype='band')
    y = filtfilt(b, a, data)
    return y

file_name = 'eog_test_sub_1_omkar_20260311_232732.csv' 
sampling_rate = 256 

try:
    df = pd.read_csv(file_name, comment='#')
    
    # 1. Filtering & Centering
    df['clean_ch0'] = bandpass_filter(df['raw_ch0'], 0.5, 30.0, sampling_rate)
    df['clean_ch1'] = bandpass_filter(df['raw_ch1'], 0.5, 30.0, sampling_rate)
    df['clean_ch0'] = df['clean_ch0'] - df['clean_ch0'].mean()
    df['clean_ch1'] = df['clean_ch1'] - df['clean_ch1'].mean()

    # 2. Spatial Filtering (vh and vv)
    df['vh'] = df['clean_ch1'] - df['clean_ch0']
    df['vv'] = (df['clean_ch1'] + df['clean_ch0']) / 2

    # --- NEW: Peak Detection Algorithms ---
    # distance=128 means we force the code to wait at least 0.5 seconds (128 samples) 
    # between detections so a single long blink doesn't count twice.
    
    # Find Blinks on the vv vector
    blink_peaks, _ = find_peaks(df['vv'], height=BLINK_THRESHOLD, distance=128)
    
    # Find Right Movements on the vh vector (positive peaks)
    right_peaks, _ = find_peaks(df['vh'], height=MOVEMENT_THRESHOLD, distance=128)
    
    # Find Left Movements on the vh vector (negative peaks, so we invert it with a minus sign)
    left_peaks, _ = find_peaks(-df['vh'], height=MOVEMENT_THRESHOLD, distance=128)

    # Put all detected events into a single list
    raw_events = []
    for p in blink_peaks: raw_events.append({'time': df['elapsed_ms'].iloc[p], 'cmd': 'Blink'})
    for p in right_peaks: raw_events.append({'time': df['elapsed_ms'].iloc[p], 'cmd': 'Looked Right'})
    for p in left_peaks: raw_events.append({'time': df['elapsed_ms'].iloc[p], 'cmd': 'Looked Left'})

    # Sort the events in chronological order
    raw_events.sort(key=lambda x: x['time'])

    # --- NEW: Artifact Rejection (Cleaning the list) ---
    clean_events = []
    last_blink_time = -99999 

    for event in raw_events:
        if event['cmd'] == 'Blink':
            last_blink_time = event['time']
            clean_events.append(event)
        else:
            # Check if this L/R movement happened within 400ms of a blink
            # If it did, it's just blink noise bleeding into the vh channel. Ignore it!
            if abs(event['time'] - last_blink_time) > 400:
                clean_events.append(event)

    # --- PRINT THE FINAL COMMAND LIST ---
    print("\n" + "="*40)
    print("🧠 FINAL BCI COMMAND LIST")
    print("="*40)
    if not clean_events:
        print("No commands detected. (Check if your thresholds are too high!)")
    else:
        for i, e in enumerate(clean_events):
            time_sec = e['time'] / 1000  # Convert ms to seconds for readability
            print(f"{i+1}. {e['cmd']:<15} at {time_sec:.2f} seconds")
    print("="*40 + "\n")

except Exception as e:
    print(f"❌ Error: {e}")