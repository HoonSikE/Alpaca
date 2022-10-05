package com.example.taxi.ui.call_taxi.setting

import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.data.dto.user.destination.DestinationSearch
import com.example.taxi.databinding.ItemDestinationSearchListBinding

class DestinationSearchListAdapter : RecyclerView.Adapter<DestinationSearchListAdapter.DestinationSearchListViewHolder>() {
    private var destinationList = listOf<DestinationSearch>()
    lateinit var onItemClickListener: (View, String, String, String, String) -> Unit

    fun setListData(data: List<DestinationSearch>){
        destinationList = data
    }

    fun updateList(list: List<DestinationSearch>){
        this.destinationList = list
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): DestinationSearchListViewHolder {
        return DestinationSearchListViewHolder(
            ItemDestinationSearchListBinding.inflate(
                LayoutInflater.from(parent.context),
                parent,
                false
            )
        ).apply {
            bindOnItemClickListener(onItemClickListener)
        }
    }

    override fun onBindViewHolder(holder: DestinationSearchListViewHolder, position: Int) {
        holder.bind(destinationList[position])
    }

    override fun getItemCount(): Int {
        return destinationList.size
    }

    class DestinationSearchListViewHolder(private val binding: ItemDestinationSearchListBinding) :
        RecyclerView.ViewHolder(binding.root) {

        private lateinit var place : String
        private lateinit var address : String
        lateinit var latitude : String
        lateinit var longitude : String

        fun bind(data: DestinationSearch) {
            binding.placeName.text = data.place_name
            binding.addressName.text = data.address_name
            binding.addressWidth.text = data.distance
            place = data.place_name
            address = data.address_name
            latitude = data.x
            longitude = data.y
        }

        fun bindOnItemClickListener(onItemClickListener: (View, String, String, String, String) -> Unit ) {
            binding.root.setOnClickListener {
                onItemClickListener(it, place, address, latitude, longitude)
            }
        }
    }

}