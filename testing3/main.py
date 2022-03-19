import testing3
if __name__ == "__main__":
    import time
    s = time.perf_counter()
    model,caps = initialize()
    eject(model,caps)
    run(model,caps)
    elapsed = time.perf_counter() - s
    print(f"{__file__} executed in {elapsed:0.2f} seconds.")
