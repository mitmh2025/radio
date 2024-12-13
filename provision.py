#!/usr/bin/env python


import os
import subprocess

import pexpect.fdpexpect
import serial
import questionary
import requests


def provision_device(server: str, auth_token: str, device_profile_id: str):
    mac = None
    port = None
    mac_output = subprocess.check_output(["esptool.py", "read_mac"])
    lines = mac_output.decode().split("\n")
    for line in lines:
        if line.startswith("MAC: "):
            mac = line.split(" ")[1].lower()
        if line.startswith("Serial port "):
            port = line.split(" ")[2]
    if not mac:
        raise ValueError("Error: MAC address not found.")
    if not port:
        raise ValueError("Error: Serial port not found.")

    print(f"Provisioning device with MAC address: {mac}")

    serial_number = questionary.text("Enter serial number").unsafe_ask()
    if not serial_number:
        raise ValueError("Serial number is required.")

    device_id: str | None = None

    # See if device is already provisioned
    response = requests.get(
        f"https://{server}/api/tenant/devices",
        params={"deviceName": mac},
        headers={"Authorization": auth_token, "Accept": "application/json"},
    )
    if response.status_code == 200:
        print("Device already provisioned.")
        device = response.json()
        device_id = device["id"]["id"]

    # Create or update device
    request_data = {
        "deviceProfileId": {
            "id": device_profile_id,
            "entityType": "DEVICE_PROFILE",
        },
        "name": mac,
        "label": serial_number,
    }
    if device_id:
        request_data["id"] = {
            "id": device_id,
            "entityType": "DEVICE",
        }

    response = requests.post(
        f"https://{server}/api/device",
        headers={
            "Authorization": auth_token,
            "Accept": "application/json",
            "Content-Type": "application/json",
        },
        json=request_data,
    )
    response.raise_for_status()

    device_id = response.json()["id"]["id"]

    response = requests.get(
        f"https://{server}/api/device/{device_id}/credentials",
        headers={
            "Authorization": auth_token,
            "Accept": "application/json",
        },
    )
    response.raise_for_status()

    credentials = response.json()
    token = credentials["credentialsId"]

    # Flash the device
    print("Flashing device...")
    # Strip out the virtualenv to get back to the original IDF environment
    env = os.environ.copy()
    venv = env.get("VIRTUAL_ENV")
    if venv:
        env["PATH"] = env["PATH"].replace(f"{venv}/bin:", "")
        env.pop("VIRTUAL_ENV")
    subprocess.check_call(
        ["idf.py", "flash", "--port", port],
        env=env,
    )

    # Provision the device
    while True:
        try:
            with serial.Serial(port, 115200, timeout=5) as ser:
                ser.dtr = False
                ser.rts = False
                ser.rts = True
                ser.rts = False
                child = pexpect.fdpexpect.fdspawn(ser, timeout=5)
                child.expect("radio>")
                child.sendline(f"provision {server} {token}")
                child.expect("radio>")
            break
        except (Exception, KeyboardInterrupt) as e:
            print(f"Error: {e}")
            retry = questionary.confirm("Retry storing token?").unsafe_ask()
            if not retry:
                raise

    print(
        "Device provisioned. Follow instructions for calibration. Press Ctrl-] to exit."
    )
    subprocess.check_call(["idf.py", "monitor", "--port", port], env=env)


def main():
    if "IDF_PATH" not in os.environ:
        print("Error: ESP-IDF not found. Please set the IDF_PATH environment variable.")
        return

    server = os.environ.get("THINGSBOARD_SERVER")
    if not server:
        server = questionary.text(
            "ThingsBoard Server URL [things.staging.mitmh2025.com]"
        ).unsafe_ask()
    if not server:
        server = "things.staging.mitmh2025.com"

    auth_token = os.environ.get("THINGSBOARD_AUTH_TOKEN")
    if not auth_token:
        auth_token = questionary.text("ThingsBoard JWT Auth Token").unsafe_ask()
    if not auth_token:
        print("Error: JWT Auth Token is required.")
        return

    print("Verifying auth token...")
    response = requests.get(
        f"https://{server}/api/auth/user",
        headers={"Authorization": auth_token, "Accept": "application/json"},
    )
    response.raise_for_status()

    device_profile_id = os.environ.get("THINGSBOARD_DEVICE_PROFILE_ID")
    if not device_profile_id:
        response = requests.get(
            f"https://{server}/api/deviceProfile/names",
            headers={"Authorization": auth_token, "Accept": "application/json"},
        )
        response.raise_for_status()
        device_profiles = response.json()
        choices = [
            questionary.Choice(str(profile["name"]), value=profile["id"]["id"])
            for profile in device_profiles
        ]
        device_profile_id = questionary.select(
            "Select device profile", choices=choices
        ).unsafe_ask()

    while True:
        try:
            if not questionary.confirm(
                "Press ENTER to provision currently connected device"
            ).unsafe_ask():
                break
            provision_device(server, auth_token, device_profile_id)
        except (Exception, KeyboardInterrupt) as e:
            print(f"Error: {e}")
            retry = questionary.confirm("Retry provisioning?").unsafe_ask()
            if not retry:
                break


if __name__ == "__main__":
    main()
