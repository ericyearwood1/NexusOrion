/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 * All rights reserved.
 *
 * Licensed under the Oculus SDK License Agreement (the "License");
 * you may not use the Oculus SDK except in compliance with the License,
 * which is provided at the time of installation or download, or which
 * otherwise accompanies this software in either electronic or hard copy form.
 *
 * You may obtain a copy of the License at
 *
 * https://developer.oculus.com/licenses/oculussdk/
 *
 * Unless required by applicable law or agreed to in writing, the Oculus SDK
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

using System;
using System.Collections.Generic;

partial struct OVRAnchor
{
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_persistence
    static readonly unsafe DeferredTaskSystem<OVRResult<SaveResult>> s_deferredSaves = new((count, spaces)
        => SaveSpacesAsync(spaces, count));

    static readonly unsafe DeferredTaskSystem<OVRResult<EraseResult>> s_deferredErases = new((count, spaces)
        => EraseSpacesAsync(count, spaces, 0, null));
#endif

    // Invoked by OVRManager.LateUpdate
    internal static void OnLateUpdate()
    {
#if OVR_PARTNER_CODE || OVR_INTERNAL_CODE // XR_META_spatial_entity_persistence
        s_deferredSaves.Execute();
        s_deferredErases.Execute();
#endif
    }

    readonly struct DeferredTaskSystem<T>
    {
        public unsafe delegate OVRTask<T> TaskGenerator(int count, ulong* spaces);

        readonly struct Entry
        {
            public readonly OVRAnchor Anchor;
            public readonly OVRTask<T> Task;
            public Entry(OVRAnchor anchor, OVRTask<T> task)
            {
                Anchor = anchor;
                Task = task;
            }
        }

        readonly List<Entry> _entries;

        readonly TaskGenerator _taskGenerator;

        public void Clear() => _entries.Clear();

        public OVRTask<T> Add(OVRAnchor anchor)
        {
            var task = OVRTask.FromGuid<T>(Guid.NewGuid());
            _entries.Add(new Entry(anchor, task));
            return task;
        }

        public DeferredTaskSystem(TaskGenerator taskGenerator)
        {
            _entries = new();
            _taskGenerator = taskGenerator;
        }

        /// <summary>
        /// Processes current entries and awaits results
        /// </summary>
        public void Execute()
        {
            if (_entries.Count > 0)
            {
                Process();
            }
        }

        async void Process()
        {
            OVRTask<T> task;
            unsafe
            {
                var spaces = stackalloc ulong[_entries.Count];
                var count = 0;
                foreach (var deferredTask in _entries)
                {
                    spaces[count++] = deferredTask.Anchor.Handle;
                }

                // Initiate the operation
                task = _taskGenerator(count, spaces);
            }

            using (new OVRObjectPool.ListScope<OVRTask<T>>(out var pendingTasks))
            {
                foreach (var entry in _entries)
                {
                    pendingTasks.Add(entry.Task);
                }

                // _entries can be re-used for the next frame
                _entries.Clear();

                // Wait for the task to complete
                var result = await task;

                // Complete all the child tasks
                foreach (var pendingTask in pendingTasks)
                {
                    pendingTask.SetResult(result);
                }
            }
        }
    }
}
