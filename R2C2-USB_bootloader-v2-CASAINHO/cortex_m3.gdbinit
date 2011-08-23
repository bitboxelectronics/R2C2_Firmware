define hook-stop
mon cortex_m3 maskisr on
end
define hookpost-stop
mon cortex_m3 maskisr off
end

define hook-step
mon cortex_m3 maskisr on
end
define hookpost-step
mon cortex_m3 maskisr off
end