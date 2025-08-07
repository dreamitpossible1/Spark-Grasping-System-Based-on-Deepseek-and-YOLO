#ght (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
import sys
import signal
import argparse
import re

import gi
gi.require_version('Gst', '1.0')
gi.require_version("GLib", "2.0")
from gi.repository import Gst, GLib

# Constants
DESCRIPTION = """
This app sets up GStreamer pipeline for object detection.
Initializes and links elements for capturing live stream from camera or offline source,
performs inference (object detection) using MODEL and LABELS files,
and renders video on display or dumps it as OUTPUT.
"""
DELEGATE_PATH = "libQnnTFLiteDelegate.so"
OUTPUT_FILE = "detected_objects.txt"  # File to store current objects

# Framework
SNPE = 1
TFLITE = 2

ML_PLUGINS = {
    SNPE   : 'qtimlsnpe',
    TFLITE : 'qtimltflite'
}

waiting_for_eos = False
eos_received = False

def extract_label(text):
    """Extract object labels from detection results."""
    # Find all matches of labels in the format "label,..."
    matches = re.finditer(r'"([^,\\]+?)\\+?,', text)
    labels = []
    for match in matches:
        label = match.group(1)
        # Skip the "ObjectDetection" structure name
        if label != "ObjectDetection":
            labels.append(label)
    return labels

def update_current_objects(current_labels):
    """Update the file with current objects and display them."""
    # Write current labels to file
    with open(OUTPUT_FILE, 'w') as f:  # Use 'w' to overwrite with current objects
        for label in current_labels:
            f.write(f"{label}\n")
    
    # Display current objects in terminal
    if current_labels:
        print("\nCurrent objects detected:", ", ".join(current_labels))
    else:
        print("\nNo objects currently detected")

def handle_interrupt_signal(pipeline, mloop):
    """Handle Ctrl+C."""
    global waiting_for_eos
    _, state, _ = pipeline.get_state(Gst.CLOCK_TIME_NONE)
    if state != Gst.State.PLAYING or waiting_for_eos:
        mloop.quit()
        return GLib.SOURCE_CONTINUE

    event = Gst.Event.new_eos()
    if pipeline.send_event(event):
        print("EoS sent to the pipeline")
        waiting_for_eos = True
    else:
        print("Failed to send EoS event to the pipeline!")
        mloop.quit()
    return GLib.SOURCE_CONTINUE

def handle_bus_message(bus, message, mloop):
    """Handle messages posted on pipeline bus."""
    global eos_received

    if message.type == Gst.MessageType.ERROR:
        error, debug_info = message.parse_error()
        print("ERROR:", message.src.get_name(), " ", error.message)
        if debug_info:
            print("debugging info:", debug_info)
        mloop.quit()
    elif message.type == Gst.MessageType.EOS:
        print("EoS received")
        eos_received = True
        mloop.quit()
    return True

def print_detection_results(sink, buffer, info, data):
    """Callback function to print detection results and update current objects."""
    try:
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            # Convert memoryview to bytes before decoding
            bytes_data = bytes(map_info.data)
            text = bytes_data.decode('utf-8')
            
            # Extract labels
            current_labels = extract_label(text)
            # Parse coordinates for center calculation
            center_info = []
            if current_labels:
                # Updated pattern to match the actual format with multiple backslashes
                coord_pattern = r'rectangle\\*=\\*\(float\\*\)\\*<\\*\s*([^>]+?)\\*\s*>\\*'
                coord_matches = re.findall(coord_pattern, text)
                
                for i, label in enumerate(current_labels):
                    if i < len(coord_matches):
                        try:
                            coord_str = re.sub(r'\\+', '', coord_str)
                            coord_str = coord_str.replace(' ', '')
                            
                            # Split by comma and convert to float
                            coords = [float(x.strip()) for x in coord_str.split(',') if x.strip()]
                            
                            if len(coords) == 4:
                                norm_x, norm_y, norm_w, norm_h = coords
                                
                                # Convert to pixel coordinates (1280x720)
                                pixel_x = int(norm_x * 1280)
                                pixel_y = int(norm_y * 720)
                                pixel_w = int(norm_w * 1280)
                                pixel_h = int(norm_h * 720)
                                
                                # Calculate center point
                                center_x = pixel_x + pixel_w // 2
                                center_y = pixel_y + pixel_h // 2
                                
                                center_info.append(f"{label}: Center({center_x}, {center_y})")
                            else:
                                center_info.append(f"{label}: Center(0, 0)")
                        except Exception as e:
                            center_info.append(f"{label}: Center(0, 0)")
                    else:
                        center_info.append(f"{label}: Center(0, 0)")
            
            # Print current objects with center points
            if center_info:
                print("Current objects detected:", ", ".join(center_info))
            else:
                print("No objects currently detected")
            
            # Write to file
            with open(OUTPUT_FILE, 'w') as f:
                for info in center_info:
                    f.write(f"{info}\n")
                
            buffer.unmap(map_info)
        return Gst.FlowReturn.OK
    except Exception as e:
        print(f"Error in print_detection_results: {e}")
        return Gst.FlowReturn.ERROR

def create_element(factory_name, name):
    """Create a GStreamer element."""
    element = Gst.ElementFactory.make(factory_name, name)
    if not element:
        raise Exception(f"Unable to create element {name}")
    return element

def on_pad_added(_, pad, target):
    """Link dynamic pads from demuxer to target element."""
    if "video" in pad.get_name():
        sink_pad = target.get_static_pad("sink")
        if not sink_pad.is_linked():
            if pad.link(sink_pad) != Gst.PadLinkReturn.OK:
                raise Exception(f"Failed linking demux to queue")

def link_elements(link_orders, elements):
    """Link elements in the specified order."""
    for link_order in link_orders:
        src = None  # Initialize src to None at the start of each link_order
        for element in link_order:
            dest = elements[element]
            if src:
                if src.get_name() == "demux":
                    src.connect("pad-added", on_pad_added, dest)
                elif not src.link(dest):
                    raise Exception(
                        "Unable to link element "
                        f"{src.get_name()} to {dest.get_name()}"
                    )
            src = dest  # Update src to the current dest for the next iteration

def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description=DESCRIPTION,
        formatter_class=type(
            'CustomFormatter',
            (argparse.ArgumentDefaultsHelpFormatter, argparse.RawTextHelpFormatter),
            {}
        )
    )

    parser.add_argument(
        '-c', '--camera', type=int, choices=[0, 1], default=0,
        help='Select (0) for Primary Camera and (1) for Secondary Camera.\n'
    )
    parser.add_argument(
        '-cw', '--width', type=int, default=1280,
        help='Camera Output Width'
    )
    parser.add_argument(
        '-ch', '--height', type=int, default=720,
        help='Camera Output Height'
    )
    parser.add_argument(
        '-cf', '--framerate', type=str, default='30/1',
        help='Camera Output Framerate (fraction)'
    )

    parser.add_argument(
        '-s', '--file-path', type=str,
        help='File source path. If not specified, input is taken from the camera.'
    )

    parser.add_argument(
        '-f', '--ml-framework', type=int, choices=[1, 2], required=True,
        help='Execute Model in SNPE DLC (1) or TFlite (2) format'
    )

    parser.add_argument('-ml', '--module', type=str, required=True,
                        help='The ml module to be used.')

    parser.add_argument('-m', '--model', type=str, required=True,
                        help='Path to the model file.')

    parser.add_argument('-l', '--labels', type=str, required=True,
                        help='Path to the labels file.')

    parser.add_argument('-k', '--constants', type=str,
                        help='Constants, offsets and coefficients used by the module for post-processing of incoming tensors.')

    parser.add_argument('--layers', type=str, help='Comma-separated list of layer names.')

    parser.add_argument('-p', '--threshold', type=float, default=75.0,
                        help='Optional parameter to override default threshold value. [0.0 - 100.0]')

    parser.add_argument('-o', '--output', type=str, help='Output File Path. If not specified, the output is displayed on the screen.')

    parser.add_argument('--use_cpu', action='store_true', help='Optional parameter to inference on CPU Runtime')

    parser.add_argument('--use_gpu', action='store_true', help='Optional parameter to inference on GPU Runtime')

    parser.add_argument('--use_dsp', action='store_true', default=True,
                        help='Default and optional parameter to inference on DSP Runtime')

    args = parser.parse_args()

    # Validate arguments based on ml-framework
    if args.ml_framework == SNPE and not args.layers:
        parser.error("--layers is required when --ml-framework is 1 (SNPE)")
    if args.ml_framework == TFLITE and not args.constants:
        parser.error("--constants is required when --ml-framework is 2 (TFLite)")

    # Convert the layers argument to a list if provided
    if args.layers:
        args.layers = [layer.strip() for layer in args.layers.split(",")]

    return args

def create_source_elements(args):
    """Create source elements based on input type."""
    if args.file_path:
        source_elements = {
            "filesrc": create_element("filesrc", "src"),
            "demux": create_element("qtdemux", "demux"),
            "parse": create_element("h264parse", "parse"),
            "decoder": create_element("v4l2h264dec", "decoder")
        }
        source_elements["filesrc"].set_property("location", args.file_path)
        source_elements["decoder"].set_property("capture-io-mode", 5)
        source_elements["decoder"].set_property("output-io-mode", 5)
    else:
        source_elements = {
            "camsrc": create_element("qtiqmmfsrc", "camsrc"),
            "camcaps": create_element("capsfilter", "camcaps")
        }
        source_elements["camsrc"].set_property("camera", args.camera)
        source_elements["camcaps"].set_property(
            "caps", Gst.Caps.from_string(
                "video/x-raw(memory:GBM),format=NV12,"
                f"width={args.width},height={args.height},"
                f"framerate={args.framerate},compression=ubwc"
            )
        )
    return source_elements

def create_sink_elements(args):
    """Create sink elements based on output type."""
    if args.output:
        sink_elements = {
            "encoder": create_element("v4l2h264enc", "encoder"),
            "parser": create_element("h264parse", "parser"),
            "mux": create_element("mp4mux", "mux"),
            "sink": create_element("filesink", "sink")
        }
        sink_elements["encoder"].set_property("capture-io-mode", 5)
        sink_elements["encoder"].set_property("output-io-mode", 5)
        sink_elements["sink"].set_property("location", args.output)
    else:
        sink_elements = {
            "display": create_element("fakesink", "display")
        }
    return sink_elements

def set_snpe_properties(elements, args):
    """Set properties for SNPE element."""
    if args.use_cpu:
        elements["ml"].set_property("delegate", "none")
        print("Using CPU delegate")
    elif args.use_gpu:
        elements["ml"].set_property("delegate", "gpu")
        print("Using GPU delegate")
    elif args.use_dsp:
        elements["ml"].set_property("delegate", "dsp")
        print("Using DSP delegate")

    elements["ml"].set_property("layers", args.layers)

def set_tflite_properties(elements, args):
    """Set properties for TFLite element."""
    if args.use_cpu:
        elements["ml"].set_property("delegate", "none")
        print("Using CPU delegate")
    elif args.use_gpu:
        elements["ml"].set_property("delegate", "gpu")
        print("Using GPU delegate")
    elif args.use_dsp:
        print("Using DSP delegate")
        delegate_options = Gst.Structure.new_empty("QNNExternalDelegate")
        delegate_options.set_value("backend_type", "htp")
        elements["ml"].set_property("delegate", "external")
        elements["ml"].set_property("external-delegate-path", DELEGATE_PATH)
        elements["ml"].set_property("external-delegate-options", delegate_options)

    elements["detection"].set_property("constants", args.constants)


def create_link_orders(args):
    """Create link orders based on source and sink types."""
    link_orders = [
        # First branch: ML processing for text output
        ["split", "queue1", "converter", "queue2", "ml", "detection", "capsfilter", "textsink"],
        # Second branch: ML processing for overlay
        ["split", "queue8", "converter2", "queue9", "ml2", "detection2", "capsfilter2", "metamux"],
        # Third branch: Original video
        ["split", "queue3", "metamux", "queue4", "overlay"]
    ]

    if args.file_path:
        link_orders.append(["filesrc", "demux", "parse", "decoder", "queue0", "split"])
    else:
        link_orders.append(["camsrc", "camcaps", "queue0", "split"])

    if args.output:
        link_orders.append(["overlay", "queue5", "encoder", "parser", "queue6", "mux", "queue7", "sink"])
    else:
        link_orders.append(["overlay", "display"])

    return link_orders

def create_pipeline(pipeline, args):
    """Initialize and link elements for the GStreamer pipeline."""
    # Check if model and label files are present
    if not os.path.exists(args.model):
        print(f"File {args.model} does not exist")
        sys.exit(1)
    if not os.path.exists(args.labels):
        print(f"File {args.model} does not exist")
        sys.exit(1)

    # Create elements
    elements = {
        "split"     : create_element("tee", "split"),
        "metamux"   : create_element("qtimetamux", "metamux"),
        "overlay"   : create_element("qtioverlay", "overlay"),
        "converter" : create_element("qtimlvconverter", "converter"),
        "converter2": create_element("qtimlvconverter", "converter2"),
        "ml"        : create_element(ML_PLUGINS.get(args.ml_framework), "ml"),
        "ml2"       : create_element(ML_PLUGINS.get(args.ml_framework), "ml2"),
        "detection" : create_element("qtimlvdetection", "detection"),
        "detection2": create_element("qtimlvdetection", "detection2"),
        "capsfilter": create_element("capsfilter", "capsfilter"),
        "capsfilter2": create_element("capsfilter", "capsfilter2"),
        "textsink"  : create_element("fakesink", "textsink")
    }

    # Set up text sink
    elements["textsink"].set_property("sync", False)
    elements["textsink"].set_property("signal-handoffs", True)
    elements["textsink"].connect("handoff", print_detection_results, None)

    source_elements = create_source_elements(args)
    sink_elements = create_sink_elements(args)

    elements.update(source_elements)
    elements.update(sink_elements)

    queue_count = 10  # Increased for new queues
    for i in range(queue_count):
        queue_name = f"queue{i}"
        elements[queue_name] = create_element("queue", queue_name)

    # Set properties
    elements["ml"].set_property("model", args.model)
    elements["ml2"].set_property("model", args.model)

    if args.ml_framework == SNPE:
        set_snpe_properties(elements, args)
        # Set properties for ml2
        if args.use_cpu:
            elements["ml2"].set_property("delegate", "none")
        elif args.use_gpu:
            elements["ml2"].set_property("delegate", "gpu")
        elif args.use_dsp:
            elements["ml2"].set_property("delegate", "dsp")
        elements["ml2"].set_property("layers", args.layers)
    elif args.ml_framework == TFLITE:
        set_tflite_properties(elements, args)
        # Set properties for ml2
        if args.use_cpu:
            elements["ml2"].set_property("delegate", "none")
        elif args.use_gpu:
            elements["ml2"].set_property("delegate", "gpu")
        elif args.use_dsp:
            delegate_options = Gst.Structure.new_empty("QNNExternalDelegate")
            delegate_options.set_value("backend_type", "htp")
            elements["ml2"].set_property("delegate", "external")
            elements["ml2"].set_property("external-delegate-path", DELEGATE_PATH)
            elements["ml2"].set_property("external-delegate-options", delegate_options)
        elements["detection2"].set_property("constants", args.constants)

    elements["detection"].set_property("threshold", args.threshold)
    elements["detection"].set_property("results", 10)
    elements["detection"].set_property("module", args.module)
    elements["detection"].set_property("labels", args.labels)

    elements["detection2"].set_property("threshold", args.threshold)
    elements["detection2"].set_property("results", 10)
    elements["detection2"].set_property("module", args.module)
    elements["detection2"].set_property("labels", args.labels)

    elements["capsfilter"].set_property("caps",
        Gst.Caps.from_string("text/x-raw")
    )
    elements["capsfilter2"].set_property("caps",
        Gst.Caps.from_string("text/x-raw")
    )

    # Add elements to the pipeline
    for element in elements.values():
        pipeline.add(element)

    # Link elements
    link_orders = create_link_orders(args)
    link_elements(link_orders, elements)

def main():
    """Main function to set up and run the GStreamer pipeline."""

    # Set the environment
    os.environ["XDG_RUNTIME_DIR"] = "/dev/socket/weston"
    os.environ["WAYLAND_DISPLAY"] = "wayland-1"
    
    # Suppress debug messages
    os.environ["GST_DEBUG"] = "0"
    os.environ["G_MESSAGES_DEBUG"] = "0"
    os.environ["GST_DEBUG_DUMP_DOT_DIR"] = "0"
    os.environ["GST_DEBUG_NO_COLOR"] = "1"
    os.environ["GST_DEBUG_DUMP_DIR_DIR"] = "0"

    # Redirect stderr to /dev/null to suppress GBM errors
    sys.stderr = open(os.devnull, 'w')

    # Initialize GStreamer
    Gst.init(None)
    mloop = GLib.MainLoop()

    # Parse arguments
    args = parse_arguments()
    
    # Print actual resolution being used
    print(f"Using resolution: {args.width}x{args.height}")

    # Create the pipeline
    try:
        pipeline = Gst.Pipeline.new("object-detection-pipeline")
        if not pipeline:
            raise Exception(f"Unable to create object detection pipeline")
        create_pipeline(pipeline, args)
    except Exception as e:
        print(f"{e} Exiting...")
        return -1

    # Handle Ctrl+C
    interrupt_watch_id = GLib.unix_signal_add(
        GLib.PRIORITY_HIGH, signal.SIGINT, handle_interrupt_signal, pipeline, mloop
    )

    # Wait until error or EOS
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", handle_bus_message, mloop)

    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    mloop.run()

    GLib.source_remove(interrupt_watch_id)
    bus.remove_signal_watch()
    bus = None

    pipeline.set_state(Gst.State.NULL)

    mloop = None
    pipeline = None
    Gst.deinit()
    if eos_received:
        print("App execution successful")

if __name__ == "__main__":
    sys.exit(main())
